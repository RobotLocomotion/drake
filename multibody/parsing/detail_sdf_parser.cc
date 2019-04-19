#include "drake/multibody/parsing/detail_sdf_parser.h"

#include <limits>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include <sdf/sdf.hh>

#include "drake/geometry/geometry_instance.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/detail_ignition.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/parsing/detail_scene_graph.h"
#include "drake/multibody/tree/fixed_offset_frame.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/tree/weld_joint.h"

namespace drake {
namespace multibody {
namespace detail {

using Eigen::Matrix3d;
using Eigen::Translation3d;
using Eigen::Vector3d;
using geometry::GeometryInstance;
using geometry::SceneGraph;
using math::RigidTransformd;
using std::unique_ptr;

// Unnamed namespace for free functions local to this file.
namespace {
// Given an ignition::math::Inertial object, extract a RotationalInertia object
// for the rotational inertia of body B, about its center of mass Bcm and,
// expressed in the inertial frame Bi (as specified in <inertial> in the SDF
// file.)
RotationalInertia<double> ExtractRotationalInertiaAboutBcmExpressedInBi(
    const ignition::math::Inertiald &inertial) {
  // TODO(amcastro-tri): Verify that ignition::math::Inertial::MOI() ALWAYS is
  // expresed in the body frame B, regardless of how a user might have
  // specified frames in the sdf file. That is, that it always returns R_BBcm_B.
  // TODO(amcastro-tri): Verify that ignition::math::Inertial::MassMatrix()
  // ALWAYS is in the inertial frame Bi, regardless of how a user might have
  // specified frames in the sdf file. That is, that it always returns
  // M_BBcm_Bi.
  const ignition::math::Matrix3d I = inertial.MassMatrix().Moi();
  return RotationalInertia<double>(I(0, 0), I(1, 1), I(2, 2),
                                   I(1, 0), I(2, 0), I(2, 1));
}

// Fails fast if a user attempts to specify `<pose frame='...'/>` in an
// unsupported location.
// See https://bitbucket.org/osrf/sdformat/issues/200 (tracked by #10590).
void ThrowIfPoseFrameSpecified(sdf::ElementPtr element) {
  if (element->HasElement("pose")) {
    sdf::ElementPtr pose = element->GetElement("pose");
    const std::string frame_name = pose->Get<std::string>("frame");
    if (!frame_name.empty()) {
      throw std::runtime_error(
          "<pose frame='{non-empty}'/> is presently not supported outside of "
          "the <frame/> tag.");
    }
  }
}

// Helper method to extract the SpatialInertia M_BBo_B of body B, about its body
// frame origin Bo and, expressed in body frame B, from an ignition::Inertial
// object.
SpatialInertia<double> ExtractSpatialInertiaAboutBoExpressedInB(
    const ignition::math::Inertiald& Inertial_BBcm_Bi) {
  double mass = Inertial_BBcm_Bi.MassMatrix().Mass();

  const RotationalInertia<double> I_BBcm_Bi =
      ExtractRotationalInertiaAboutBcmExpressedInBi(Inertial_BBcm_Bi);

  // Pose of the "<inertial>" frame Bi in the body frame B.
  // TODO(amcastro-tri): Verify we don't get funny results when X_BBi is not
  // the identity matrix.
  // TODO(amcastro-tri): SDF seems to provide <inertial><frame/></inertial>
  // together with <inertial><pose/></inertial>. It'd seem then the frame B
  // in X_BI could be another frame. We rely on Inertial::Pose() to ALWAYS
  // give us X_BI. Verify this.
  const RigidTransformd X_BBi = ToRigidTransform(Inertial_BBcm_Bi.Pose());

  // B and Bi are not necessarily aligned.
  const math::RotationMatrix<double> R_BBi(X_BBi.linear());

  // Re-express in frame B as needed.
  const RotationalInertia<double> I_BBcm_B = I_BBcm_Bi.ReExpress(R_BBi);

  // Bi's origin is at the COM as documented in
  // http://sdformat.org/spec?ver=1.6&elem=link#inertial_pose
  const Vector3d p_BoBcm_B = X_BBi.translation();

  // Return the spatial inertia M_BBo_B of body B, about its body frame origin
  // Bo, and expressed in the body frame B.
  return SpatialInertia<double>::MakeFromCentralInertia(
      mass, p_BoBcm_B, I_BBcm_B);
}

// Helper method to retrieve a Body given the name of the link specification.
const Body<double>& GetBodyByLinkSpecificationName(
    const sdf::Model& model, const std::string& link_name,
    ModelInstanceIndex model_instance, const MultibodyPlant<double>& plant) {
  // SDF's convention to indicate a joint is connected to the world is to either
  // name the corresponding link "world" or just leave it unnamed.
  // Thus this the "if" statement in the following line.
  if (link_name.empty() || link_name == "world") {
    return plant.world_body();
  } else {
    // TODO(amcastro-tri): Remove this when sdformat guarantees a model
    // with basic structural checks.
    if (!model.LinkNameExists(link_name)) {
      throw std::logic_error("There is no parent link named '" +
          link_name + "' in the model.");
    }
    return plant.GetBodyByName(link_name, model_instance);
  }
}

// Extracts a Vector3d representation of the joint axis for joints with an axis.
Vector3d ExtractJointAxis(const sdf::Model& model_spec,
                          const sdf::Joint& joint_spec) {
  DRAKE_DEMAND(joint_spec.Type() == sdf::JointType::REVOLUTE ||
      joint_spec.Type() == sdf::JointType::PRISMATIC);

  // Axis specification.
  const sdf::JointAxis* axis = joint_spec.Axis();
  if (axis == nullptr) {
    throw std::runtime_error(
        "An axis must be specified for joint '" + joint_spec.Name() + "'");
  }

  // Joint axis, by default in the joint frame J.
  // TODO(amcastro-tri): Verify JointAxis::UseParentModelFrame is actually
  // supported by sdformat.
  Vector3d axis_J = ToVector3(axis->Xyz());
  if (axis->UseParentModelFrame()) {
    // Pose of the joint frame J in the frame of the child link C.
    ThrowIfPoseFrameSpecified(joint_spec.Element());
    const RigidTransformd X_CJ = ToRigidTransform(joint_spec.Pose());
    // Get the pose of the child link C in the model frame M.
    const RigidTransformd X_MC = ToRigidTransform(
        model_spec.LinkByName(joint_spec.ChildLinkName())->Pose());
    const RigidTransformd X_MJ = X_MC * X_CJ;
    // axis_J actually contains axis_M, expressed in the model frame M.
    axis_J = X_MJ.linear().transpose() * axis_J;
  }
  return axis_J;
}

// Helper to parse the damping for a given joint specification.
// Right now we only parse the <damping> tag.
// An exception is thrown if the provided damping value is negative or if there
// is no <axis> under <joint>.
double ParseJointDamping(const sdf::Joint& joint_spec) {
  DRAKE_DEMAND(joint_spec.Type() == sdf::JointType::REVOLUTE ||
      joint_spec.Type() == sdf::JointType::PRISMATIC);

  // Axis specification.
  const sdf::JointAxis* axis = joint_spec.Axis();
  if (axis == nullptr) {
    throw std::runtime_error(
        "An axis must be specified for joint '" + joint_spec.Name() + "'");
  }
  const double damping = axis->Damping();
  if (damping < 0) {
    throw std::runtime_error(
        "Joint damping is negative for joint '" + joint_spec.Name() + "'. "
            "Joint damping must be a non-negative number.");
  }
  return damping;
}

// Extracts the effort limit from a joint specification and adds an actuator if
// the value is non-zero. In SDF, effort limits are specified in
// <joint><axis><limit><effort>. In Drake, we understand that joints with an
// effort limit of zero are not actuated.
// Only available for "revolute" and "prismatic" joints.
void AddJointActuatorFromSpecification(
    const sdf::Joint &joint_spec, const Joint<double>& joint,
    MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(joint_spec.Type() == sdf::JointType::REVOLUTE ||
      joint_spec.Type() == sdf::JointType::PRISMATIC);

  // Axis specification.
  const sdf::JointAxis* axis = joint_spec.Axis();
  if (axis == nullptr) {
    throw std::runtime_error(
        "An axis must be specified for joint '" + joint_spec.Name() + "'");
  }

  double max_effort = axis->Effort();

  // The SDF specification defines this max_effort = -1 when no limit is
  // provided (a non-zero value). In Drake we interpret a value of exactly zero
  // as a way to specify un-actuated joints. Thus, the user would say
  // <effort>0</effort> for un-actuated joints.
  if (max_effort != 0) {
    // TODO(amcastro-tri): For positive max_effort values, store it and use it
    // to limit input actuation.
    plant->AddJointActuator(joint_spec.Name(), joint);
  }
}

// Returns joint limits as the tuple (lower_limit, upper_limit,
// velocity_limit).  The units of the limits depend on the particular joint
// type.  For prismatic joints, units are meters for the position limits and
// m/s for the velocity limit.  For revolute joints, units are radians for the
// position limits and rad/s for the velocity limit.  The velocity limit is
// always >= 0.  This method throws an exception if the joint type is not one
// of revolute or prismatic.
std::tuple<double, double, double> ParseJointLimits(
    const sdf::Joint& joint_spec) {
  DRAKE_THROW_UNLESS(joint_spec.Type() == sdf::JointType::REVOLUTE ||
      joint_spec.Type() == sdf::JointType::PRISMATIC);
  // Axis specification.
  const sdf::JointAxis* axis = joint_spec.Axis();
  if (axis == nullptr) {
    throw std::runtime_error(
        "An axis must be specified for joint '" + joint_spec.Name() + "'");
  }

  // SDF defaults to ±1.0e16 for joints with no limits, see
  // http://sdformat.org/spec?ver=1.6&elem=joint#axis_limit.
  // Drake marks joints with no limits with ±numeric_limits<double>::infinity()
  // and therefore we make the change here.
  const double lower_limit =
      axis->Lower() == -1.0e16 ?
      -std::numeric_limits<double>::infinity() : axis->Lower();
  const double upper_limit =
      axis->Upper() == 1.0e16 ?
      std::numeric_limits<double>::infinity() : axis->Upper();
  if (lower_limit > upper_limit) {
    throw std::runtime_error(
        "The lower limit must be lower (or equal) than the upper limit for "
        "joint '" + joint_spec.Name() + "'.");
  }

  // SDF defaults to -1.0 for joints with no limits, see
  // http://sdformat.org/spec?ver=1.6&elem=joint#limit_velocity
  // Drake marks joints with no limits with ±numeric_limits<double>::infinity()
  // and therefore we make the change here.
  const double velocity_limit =
      axis->MaxVelocity() == -1.0 ?
      std::numeric_limits<double>::infinity() : axis->MaxVelocity();
  if (velocity_limit < 0) {
    throw std::runtime_error(
        "Velocity limit is negative for joint '" + joint_spec.Name() + "'. "
            "Velocity limit must be a non-negative number.");
  }

  return std::make_tuple(lower_limit, upper_limit, velocity_limit);
}

// Helper method to add joints to a MultibodyPlant given an sdf::Joint
// specification object.
void AddJointFromSpecification(
    const sdf::Model& model_spec, const sdf::Joint& joint_spec,
    ModelInstanceIndex model_instance, MultibodyPlant<double>* plant) {
  // Pose of the model frame M in the world frame W.
  const RigidTransformd X_WM = ToRigidTransform(model_spec.Pose());

  const Body<double>& parent_body = GetBodyByLinkSpecificationName(
      model_spec, joint_spec.ParentLinkName(), model_instance, *plant);
  const Body<double>& child_body = GetBodyByLinkSpecificationName(
      model_spec, joint_spec.ChildLinkName(), model_instance, *plant);

  // Get the pose of frame J in the frame of the child link C, as specified in
  // <joint> <pose> ... </pose></joint>.
  // TODO(eric.cousineau): Verify sdformat supports frame specifications
  // correctly.
  ThrowIfPoseFrameSpecified(joint_spec.Element());
  const RigidTransformd X_CJ = ToRigidTransform(joint_spec.Pose());

  // Get the pose of the child link C in the model frame M.
  // TODO(eric.cousineau): Figure out how to use link poses when they are NOT
  // connected to a joint.
  const RigidTransformd X_MC = ToRigidTransform(
      model_spec.LinkByName(joint_spec.ChildLinkName())->Pose());

  // Pose of the joint frame J in the model frame M.
  const RigidTransformd X_MJ = X_MC * X_CJ;

  // Pose of the frame J in the parent body frame P.
  optional<RigidTransformd> X_PJ;
  // We need to treat the world case separately since sdformat does not create
  // a "world" link from which we can request its pose (which in that case would
  // be the identity).
  if (parent_body.index() == world_index()) {
    X_PJ = X_WM * X_MJ;  // Since P == W.
  } else {
    // Get the pose of the parent link P in the model frame M.
    const RigidTransformd X_MP = ToRigidTransform(
        model_spec.LinkByName(joint_spec.ParentLinkName())->Pose());
    X_PJ = X_MP.inverse() * X_MJ;
  }

  // If P and J are coincident, we won't create a new frame for J, but use frame
  // P directly. We indicate that by passing a nullopt.
  if (X_PJ.value().IsExactlyIdentity()) X_PJ = nullopt;

  // These will only be populated for prismatic and revolute joints.
  double lower_limit = 0;
  double upper_limit = 0;
  double velocity_limit = 0;

  switch (joint_spec.Type()) {
    case sdf::JointType::FIXED: {
      plant->AddJoint<WeldJoint>(
          joint_spec.Name(),
          parent_body, X_PJ,
          child_body, X_CJ,
          RigidTransformd::Identity() /* X_JpJc */);
      break;
    }
    case sdf::JointType::PRISMATIC: {
      const double damping = ParseJointDamping(joint_spec);
      Vector3d axis_J = ExtractJointAxis(model_spec, joint_spec);
      std::tie(lower_limit, upper_limit, velocity_limit) =
          ParseJointLimits(joint_spec);
      const auto& joint = plant->AddJoint<PrismaticJoint>(
          joint_spec.Name(),
          parent_body, X_PJ,
          child_body, X_CJ, axis_J, lower_limit, upper_limit, damping);
      plant->get_mutable_joint(joint.index()).set_velocity_limits(
          Vector1d(-velocity_limit), Vector1d(velocity_limit));
      AddJointActuatorFromSpecification(joint_spec, joint, plant);
      break;
    }
    case sdf::JointType::REVOLUTE: {
      const double damping = ParseJointDamping(joint_spec);
      Vector3d axis_J = ExtractJointAxis(model_spec, joint_spec);
      std::tie(lower_limit, upper_limit, velocity_limit) =
          ParseJointLimits(joint_spec);
      const auto& joint = plant->AddJoint<RevoluteJoint>(
          joint_spec.Name(),
          parent_body, X_PJ,
          child_body, X_CJ, axis_J, lower_limit, upper_limit, damping);
      plant->get_mutable_joint(joint.index()).set_velocity_limits(
          Vector1d(-velocity_limit), Vector1d(velocity_limit));
      AddJointActuatorFromSpecification(joint_spec, joint, plant);
      break;
    }
    default: {
      throw std::logic_error(
          "Joint type not supported for joint '" + joint_spec.Name() + "'.");
    }
  }
}

// Helper method to load an SDF file and read the contents into an sdf::Root
// object.
std::string LoadSdf(
    sdf::Root* root,
    const std::string& file_name) {

  const std::string full_path = GetFullPath(file_name);

  // Load the SDF file.
  sdf::Errors errors = root->Load(full_path);

  // Check for any errors.
  if (!errors.empty()) {
    std::string error_accumulation("From AddModelFromSdfFile():\n");
    for (const auto& e : errors)
      error_accumulation += "Error: " + e.Message() + "\n";
    throw std::runtime_error(error_accumulation);
  }

  // Uses the directory holding the SDF to be the root directory
  // in which to search for files referenced within the SDF file.
  std::string root_dir = ".";
  size_t found = full_path.find_last_of("/\\");
  if (found != std::string::npos) {
    root_dir = full_path.substr(0, found);
  }

  return root_dir;
}

// Helper method to add a model to a MultibodyPlant given an sdf::Model
// specification object.
void AddLinksFromSpecification(
    const ModelInstanceIndex model_instance,
    const sdf::Model& model,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph,
    const PackageMap& package_map,
    const std::string& root_dir) {

  // Add all the links
  for (uint64_t link_index = 0; link_index < model.LinkCount(); ++link_index) {
    const sdf::Link& link = *model.LinkByIndex(link_index);

    // Fail fast for `<pose frame='...'/>`.
    ThrowIfPoseFrameSpecified(link.Element());

    // Get the link's inertia relative to the Bcm frame.
    // sdf::Link::Inertial() provides a representation for the SpatialInertia
    // M_Bcm_Bi of body B, about its center of mass Bcm, and expressed in an
    // inertial frame Bi as defined in <inertial> <pose></pose> </inertial>.
    // Per SDF specification, Bi's origin is at the COM Bcm, but Bi is not
    // necessarily aligned with B.
    if (link.Element()->HasElement("inertial")) {
      ThrowIfPoseFrameSpecified(link.Element()->GetElement("inertial"));
    }
    const ignition::math::Inertiald& Inertial_Bcm_Bi = link.Inertial();

    const SpatialInertia<double> M_BBo_B =
        ExtractSpatialInertiaAboutBoExpressedInB(Inertial_Bcm_Bi);

    // Add a rigid body to model each link.
    const RigidBody<double>& body =
        plant->AddRigidBody(link.Name(), model_instance, M_BBo_B);

    if (plant->geometry_source_is_registered()) {
      for (uint64_t visual_index = 0; visual_index < link.VisualCount();
           ++visual_index) {
        const sdf::Visual sdf_visual = ResolveVisualUri(
            *link.VisualByIndex(visual_index), package_map, root_dir);
        unique_ptr<GeometryInstance> geometry_instance =
            MakeGeometryInstanceFromSdfVisual(sdf_visual);
        ThrowIfPoseFrameSpecified(sdf_visual.Element());
        // We check for nullptr in case someone decided to specify an SDF
        // <empty/> geometry.
        if (geometry_instance) {
          // The parsing should *always* produce an IllustrationProperties
          // instance, even if it is empty.
          DRAKE_DEMAND(
              geometry_instance->illustration_properties() != nullptr);

          plant->RegisterVisualGeometry(
              body, RigidTransformd(geometry_instance->pose()),
              geometry_instance->shape(), geometry_instance->name(),
              *geometry_instance->illustration_properties());
        }
      }

      for (uint64_t collision_index = 0;
           collision_index < link.CollisionCount(); ++collision_index) {
        const sdf::Collision& sdf_collision =
            *link.CollisionByIndex(collision_index);
        const sdf::Geometry& sdf_geometry = *sdf_collision.Geom();
        ThrowIfPoseFrameSpecified(sdf_collision.Element());
        if (sdf_geometry.Type() != sdf::GeometryType::EMPTY) {
          const RigidTransformd X_LG(
              MakeGeometryPoseFromSdfCollision(sdf_collision));
          std::unique_ptr<geometry::Shape> shape =
              MakeShapeFromSdfGeometry(sdf_geometry);
          const CoulombFriction<double> coulomb_friction =
              MakeCoulombFrictionFromSdfCollisionOde(sdf_collision);
          plant->RegisterCollisionGeometry(body, X_LG, *shape,
                                           sdf_collision.Name(),
                                           coulomb_friction, scene_graph);
        }
      }
    }
  }
}

template <typename T>
const Frame<T>& GetFrameOrWorldByName(
    const MultibodyPlant<T>& plant, const std::string& name,
    ModelInstanceIndex model_instance) {
  if (name == "world") {
    return plant.world_frame();
  } else {
    return plant.GetFrameByName(name, model_instance);
  }
}

void AddFramesFromSpecification(
    ModelInstanceIndex model_instance,
    sdf::ElementPtr parent_element,
    const Frame<double>& parent_frame,
    MultibodyPlant<double>* plant) {
  // Per its API documentation, `GetElement(...)` will create a new element if
  // one does not already exist rather than return `nullptr`; use
  // `HasElement(...)` instead.
  if (parent_element->HasElement("frame")) {
    sdf::ElementPtr frame_element = parent_element->GetElement("frame");
    while (frame_element) {
      std::string name = frame_element->Get<std::string>("name");
      sdf::ElementPtr pose_element = frame_element->GetElement("pose");
      const Frame<double>* pose_frame = &parent_frame;
      // SDF makes frame have a value of '' by default, even if unspecified.
      DRAKE_DEMAND(pose_element->HasAttribute("frame"));
      const std::string pose_frame_name =
          pose_element->Get<std::string>("frame");
      if (!pose_frame_name.empty()) {
        // TODO(eric.cousineau): Prevent instance name from leaking in? Throw
        // an error if a user ever specifies it?
        pose_frame = &GetFrameOrWorldByName(
            *plant, pose_frame_name, model_instance);
      }
      const RigidTransformd X_PF =
          ToRigidTransform(pose_element->Get<ignition::math::Pose3d>());
      plant->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
          name, *pose_frame, X_PF));
      frame_element = frame_element->GetNextElement("frame");
    }
  }
}

// Helper method to add a model to a MultibodyPlant given an sdf::Model
// specification object.
ModelInstanceIndex AddModelFromSpecification(
    const sdf::Model& model,
    const std::string& model_name,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph,
    const PackageMap& package_map,
    const std::string& root_dir) {

  const ModelInstanceIndex model_instance =
    plant->AddModelInstance(model_name);

  // TODO(eric.cousineau): Ensure this generalizes to cases when the parent
  // frame is not the world. At present, we assume the parent frame is the
  // world.
  ThrowIfPoseFrameSpecified(model.Element());
  const RigidTransformd X_WM = ToRigidTransform(model.Pose());
  // Add the SDF "model frame" given the model name so that way any frames added
  // to the plant are associated with this current model instance.
  // N.B. We mangle this name to dis-incentivize users from wanting to use
  // this frame. At present, SDFormat does not concretely specify what the
  // semantics of a "model frame" are. This current interpretation expects that
  // the SDF "model frame" is where the model is *added*, and thus is going to
  // be attached to the world, and cannot be welded to another body. This means
  // the SDF "model frame" will not "follow" the other link of a body if that
  // link is welded elsewhere.
  const std::string sdf_model_frame_name =
      "_" + model_name + "_sdf_model_frame";
  const Frame<double>& sdf_model_frame =
      plant->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
          sdf_model_frame_name, plant->world_frame(), X_WM, model_instance));

  // TODO(eric.cousineau): Register frames from SDF once we have a pose graph.
  AddLinksFromSpecification(
      model_instance, model, plant, scene_graph, package_map, root_dir);

  // Add all the joints
  // TODO(eric.cousineau): Register frames from SDF once we have a pose graph.
  for (uint64_t joint_index = 0; joint_index < model.JointCount();
       ++joint_index) {
    // Get a pointer to the SDF joint, and the joint axis information.
    const sdf::Joint& joint = *model.JointByIndex(joint_index);
    AddJointFromSpecification(model, joint, model_instance, plant);
  }

  // Add frames at root-level of <model>.
  // TODO(eric.cousineau): Address additional items:
  // - adding frames nested in other elements (joints, visuals, etc.)
  // - implicit frames for other elements (joints, visuals, etc.)
  // - explicitly referring to SDF model frame?
  // See: https://bitbucket.org/osrf/sdformat/issues/200
  AddFramesFromSpecification(
      model_instance, model.Element(), sdf_model_frame, plant);

  return model_instance;
}
}  // namespace

ModelInstanceIndex AddModelFromSdfFile(
    const std::string& file_name,
    const std::string& model_name_in,
    const PackageMap& package_map,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(!plant->is_finalized());

  sdf::Root root;

  std::string root_dir = LoadSdf(&root, file_name);

  if (root.ModelCount() != 1) {
    throw std::runtime_error("File must have a single <model> element.");
  }

  // Get the only model in the file.
  const sdf::Model& model = *root.ModelByIndex(0);

  if (scene_graph != nullptr && !plant->geometry_source_is_registered()) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
  }

  const std::string model_name =
      model_name_in.empty() ? model.Name() : model_name_in;

  return AddModelFromSpecification(
      model, model_name, plant, scene_graph, package_map, root_dir);
}

std::vector<ModelInstanceIndex> AddModelsFromSdfFile(
    const std::string& file_name,
    const PackageMap& package_map,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(!plant->is_finalized());

  sdf::Root root;

  std::string root_dir = LoadSdf(&root, file_name);

  // Throw an error if there are no models or worlds.
  if (root.ModelCount() == 0 && root.WorldCount() == 0) {
    throw std::runtime_error(
        "File must have at least one <model>, or <world> with one "
        "child <model> element.");
  }

  // Only one world in an SDF file is allowed.
  if (root.WorldCount() > 1) {
    throw std::runtime_error("File must contain only one <world>.");
  }

  // Do not allow a <world> to have a <model> sibling.
  if (root.ModelCount() > 0 && root.WorldCount() > 0) {
    throw std::runtime_error("A <world> and <model> cannot be siblings.");
  }

  if (scene_graph != nullptr && !plant->geometry_source_is_registered()) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
  }

  std::vector<ModelInstanceIndex> model_instances;

  // At this point there should be only Models or a single World at the Root
  // levelt.
  if (root.ModelCount() > 0) {
    // Load all the models at the root level.
    for (uint64_t i = 0; i < root.ModelCount(); ++i) {
      // Get the model.
      const sdf::Model& model = *root.ModelByIndex(i);
      model_instances.push_back(AddModelFromSpecification(
            model, model.Name(), plant, scene_graph, package_map, root_dir));
    }
  } else {
    // Load the world and all the models in the world.
    const sdf::World& world = *root.WorldByIndex(0);

    for (uint64_t model_index = 0; model_index < world.ModelCount();
        ++model_index) {
      // Get the model.
      const sdf::Model& model = *world.ModelByIndex(model_index);
      model_instances.push_back(AddModelFromSpecification(
            model, model.Name(), plant, scene_graph, package_map, root_dir));
    }
  }

  return model_instances;
}

}  // namespace detail
}  // namespace multibody
}  // namespace drake
