#include "drake/multibody/parsing/detail_sdf_parser.h"

#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include <sdf/sdf.hh>

#include "drake/geometry/geometry_instance.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_ignition.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/parsing/detail_scene_graph.h"
#include "drake/multibody/tree/fixed_offset_frame.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/revolute_spring.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/tree/weld_joint.h"

namespace drake {
namespace multibody {
namespace internal {

using Eigen::Matrix3d;
using Eigen::Translation3d;
using Eigen::Vector3d;
using geometry::GeometryInstance;
using geometry::SceneGraph;
using math::RigidTransformd;
using math::RotationMatrixd;
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
void ThrowIfPoseFrameSpecified(sdf::ElementPtr element) {
  // TODO(eric.cousineau): Fix this for `<inertial/>` and `<model/>`.
  if (element->HasElement("pose")) {
    sdf::ElementPtr pose = element->GetElement("pose");
    const std::string frame_name = pose->Get<std::string>("relative_to");
    if (!frame_name.empty()) {
      throw std::runtime_error(
          "<pose relative_to='{non-empty}'/> is presently not supported "
          "in <inertial/> or <model/> tags.");
    }
  }
}

// Throws an exception if there are any errors present in the `errors` list.
void ThrowAnyErrors(const sdf::Errors& errors) {
  if (!errors.empty()) {
    std::ostringstream os;
    os << "From AddModelFromSdfFile():";
    for (const auto& e : errors)
      os << "\nError: " + e.Message();
    throw std::runtime_error(os.str());
  }
}

// This takes an `sdf::SemanticPose`, which defines a pose relative to a frame,
// and resolves its value with respect to another frame.
math::RigidTransformd ResolveRigidTransform(
    const sdf::SemanticPose& semantic_pose,
    const std::string& relative_to = "") {
  ignition::math::Pose3d pose;
  ThrowAnyErrors(semantic_pose.Resolve(pose, relative_to));
  return ToRigidTransform(pose);
}

Eigen::Vector3d ResolveAxisXyz(const sdf::JointAxis& axis) {
  ignition::math::Vector3d xyz;
  ThrowAnyErrors(axis.ResolveXyz(xyz));
  return ToVector3(xyz);
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
  const RotationMatrixd R_BBi = X_BBi.rotation();

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
  unused(model_spec);
  DRAKE_DEMAND(joint_spec.Type() == sdf::JointType::REVOLUTE ||
      joint_spec.Type() == sdf::JointType::PRISMATIC);

  // Axis specification.
  const sdf::JointAxis* axis = joint_spec.Axis();
  if (axis == nullptr) {
    throw std::runtime_error(
        "An axis must be specified for joint '" + joint_spec.Name() + "'");
  }

  // Joint axis in the joint frame J.
  Vector3d axis_J = ResolveAxisXyz(*axis);
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

  // The effort_limit should always be non-negative. Negative effort will be
  // treated as infinity as specified by the sdf standard.
  const double effort_limit = axis->Effort() < 0
                                  ? std::numeric_limits<double>::infinity()
                                  : axis->Effort();

  // In Drake we interpret a value of exactly zero
  // as a way to specify un-actuated joints. Thus, the user would say
  // <effort>0</effort> for un-actuated joints.
  if (effort_limit != 0) {
    plant->AddJointActuator(joint_spec.Name(), joint, effort_limit);
  }
}

// Extracts the spring stiffness and the spring reference from a joint
// specification and adds a revolute spring force element with the
// corresponding spring reference if the spring stiffness is nonzero.
// Only available for "revolute" joints. The units for spring
// reference is radians and the units for spring stiffness is N⋅m/rad.
void AddRevoluteSpringFromSpecification(
    const sdf::Joint &joint_spec, const RevoluteJoint<double>& joint,
    MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(joint_spec.Type() == sdf::JointType::REVOLUTE);

  // Axis specification.
  const sdf::JointAxis* axis = joint_spec.Axis();
  if (axis == nullptr) {
    throw std::runtime_error(
      "An axis must be specified for joint '" + joint_spec.Name() + "'");
  }

  const double spring_reference = axis->SpringReference();
  const double spring_stiffness = axis->SpringStiffness();

  // We don't add a force element if stiffness is zero.
  // If a negative value is passed in, RevoluteSpring will
  // throw an error.
  if (spring_stiffness != 0) {
    plant->AddForceElement<RevoluteSpring>(
      joint, spring_reference, spring_stiffness);
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
    const sdf::Model& model_spec, const RigidTransformd& X_WM,
    const sdf::Joint& joint_spec, ModelInstanceIndex model_instance,
    MultibodyPlant<double>* plant,
    std::set<sdf::JointType>* joint_types) {
  const Body<double>& parent_body = GetBodyByLinkSpecificationName(
      model_spec, joint_spec.ParentLinkName(), model_instance, *plant);
  const Body<double>& child_body = GetBodyByLinkSpecificationName(
      model_spec, joint_spec.ChildLinkName(), model_instance, *plant);

  // Get the pose of frame J in the frame of the child link C, as specified in
  // <joint> <pose> ... </pose></joint>. The default `relative_to` pose of a
  // joint will be the child link.
  const RigidTransformd X_CJ =
      ResolveRigidTransform(joint_spec.SemanticPose());

  // Pose of the frame J in the parent body frame P.
  std::optional<RigidTransformd> X_PJ;
  // We need to treat the world case separately since sdformat does not create
  // a "world" link from which we can request its pose (which in that case would
  // be the identity).
  if (parent_body.index() == world_index()) {
    const RigidTransformd X_MJ =
        ResolveRigidTransform(joint_spec.SemanticPose(), "__model__");
    X_PJ = X_WM * X_MJ;  // Since P == W.
  } else {
    X_PJ = ResolveRigidTransform(
        joint_spec.SemanticPose(), joint_spec.ParentLinkName());
  }

  // If P and J are coincident, we won't create a new frame for J, but use frame
  // P directly. We indicate that by passing a nullopt.
  if (X_PJ.value().IsExactlyIdentity()) X_PJ = std::nullopt;

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
      AddRevoluteSpringFromSpecification(joint_spec, joint, plant);
      break;
    }
    default: {
      throw std::logic_error(
          "Joint type not supported for joint '" + joint_spec.Name() + "'.");
    }
  }
  joint_types->insert(joint_spec.Type());
}

// Helper method to load an SDF file and read the contents into an sdf::Root
// object.
std::string LoadSdf(
    sdf::Root* root,
    const std::string& file_name) {

  const std::string full_path = GetFullPath(file_name);

  // Load the SDF file.
  ThrowAnyErrors(root->Load(full_path));

  // Uses the directory holding the SDF to be the root directory
  // in which to search for files referenced within the SDF file.
  std::string root_dir = ".";
  size_t found = full_path.find_last_of("/\\");
  if (found != std::string::npos) {
    root_dir = full_path.substr(0, found);
  }

  return root_dir;
}

struct LinkInfo {
  const RigidBody<double>* body{};
  RigidTransformd X_WL;
};

// Helper method to add a model to a MultibodyPlant given an sdf::Model
// specification object.
std::vector<LinkInfo> AddLinksFromSpecification(
    const ModelInstanceIndex model_instance,
    const sdf::Model& model,
    const RigidTransformd& X_WM,
    MultibodyPlant<double>* plant,
    const PackageMap& package_map,
    const std::string& root_dir) {
  std::vector<LinkInfo> link_infos;

  // Add all the links
  for (uint64_t link_index = 0; link_index < model.LinkCount(); ++link_index) {
    const sdf::Link& link = *model.LinkByIndex(link_index);

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

    // Register information.
    const RigidTransformd X_ML = ResolveRigidTransform(link.SemanticPose());
    const RigidTransformd X_WL = X_WM * X_ML;
    link_infos.push_back(LinkInfo{&body, X_WL});

    // Set the initial pose of the free body (only use if the body is indeed
    // floating).
    plant->SetDefaultFreeBodyPose(body, X_WL);

    if (plant->geometry_source_is_registered()) {
      ResolveFilename resolve_filename =
        [&package_map, &root_dir](std::string uri) {
          const std::string resolved_name =
              ResolveUri(uri, package_map, root_dir);
          if (resolved_name.empty()) {
            throw std::runtime_error(
                "ERROR: Mesh file name could not be resolved from the "
                "provided uri \"" + uri + "\".");
          }
          return resolved_name;
      };

      for (uint64_t visual_index = 0; visual_index < link.VisualCount();
           ++visual_index) {
        const sdf::Visual& sdf_visual = *link.VisualByIndex(visual_index);
        const RigidTransformd X_LG = ResolveRigidTransform(
            sdf_visual.SemanticPose());
        unique_ptr<GeometryInstance> geometry_instance =
            MakeGeometryInstanceFromSdfVisual(
                sdf_visual, resolve_filename, X_LG);
        // We check for nullptr in case someone decided to specify an SDF
        // <empty/> geometry.
        if (geometry_instance) {
          // The parsing should *always* produce an IllustrationProperties
          // instance, even if it is empty.
          DRAKE_DEMAND(
              geometry_instance->illustration_properties() != nullptr);

          plant->RegisterVisualGeometry(
              body, geometry_instance->pose(), geometry_instance->shape(),
              geometry_instance->name(),
              *geometry_instance->illustration_properties());
        }
      }

      for (uint64_t collision_index = 0;
           collision_index < link.CollisionCount(); ++collision_index) {
        const sdf::Collision& sdf_collision =
            *link.CollisionByIndex(collision_index);
        const sdf::Geometry& sdf_geometry = *sdf_collision.Geom();

        std::unique_ptr<geometry::Shape> shape =
            MakeShapeFromSdfGeometry(sdf_geometry, resolve_filename);
        if (shape != nullptr) {
          const RigidTransformd X_LG = ResolveRigidTransform(
              sdf_collision.SemanticPose());
          const RigidTransformd X_LC =
              MakeGeometryPoseFromSdfCollision(sdf_collision, X_LG);
          geometry::ProximityProperties props =
              MakeProximityPropertiesForCollision(sdf_collision);
          plant->RegisterCollisionGeometry(body, X_LC, *shape,
                                           sdf_collision.Name(),
                                           std::move(props));
        }
      }
    }
  }
  return link_infos;
}

const Frame<double>& AddFrameFromSpecification(
    const sdf::Frame& frame_spec, ModelInstanceIndex model_instance,
    const Frame<double>& default_frame, MultibodyPlant<double>* plant) {
  const Frame<double>* parent_frame{};
  // TODO(eric.cousineau): Without supplying AttachedTo(), this ResolvePose
  // call fails. Debug why.
  const RigidTransformd X_PF = ResolveRigidTransform(
      frame_spec.SemanticPose(), frame_spec.AttachedTo());
  if (frame_spec.AttachedTo().empty()) {
    parent_frame = &default_frame;
  } else {
    parent_frame = &plant->GetFrameByName(
        frame_spec.AttachedTo(), model_instance);
  }
  const Frame<double>& frame =
      plant->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
          frame_spec.Name(), *parent_frame, X_PF));
  return frame;
}

const LinearBushingRollPitchYaw<double>& AddBushingFromSpecification(
    const sdf::ElementPtr node, MultibodyPlant<double>* plant) {
  // Functor to read a vector valued child tag with tag name: `element_name`
  // e.g. <element_name>0 0 0</element_name>
  // Throws an error if the tag does not exist or if the value is not properly
  // formatted.
  auto read_vector = [node](const char* element_name) -> Eigen::Vector3d {
    if (!node->HasElement(element_name)) {
      throw std::runtime_error(
          fmt::format("Unable to find the <{}> tag.", element_name));
    }

    auto [value, successful] = node->Get<ignition::math::Vector3d>(
        element_name, ignition::math::Vector3d() /* default value. not used */);

    if (!successful) {
      throw std::runtime_error(fmt::format(
          "Unable to read the value of the <{}> tag.", element_name));
    }

    return ToVector3(value);
  };

  // Functor to read a child tag with tag name: `element_name` that specifies a
  // frame name, e.g. <element_name>frame_name</element_name>
  // Throws an error if the tag does not exist or if the value
  // is not properly formatted.
  auto read_frame = [node,
                     plant](const char* element_name) -> const Frame<double>& {
    if (!node->HasElement(element_name)) {
      throw std::runtime_error(
          fmt::format("Unable to find the <{}> tag.", element_name));
    }

    auto [frame_name, successful] = node->Get<std::string>(
        element_name, std::string() /* default value. not used */);

    if (!successful) {
      throw std::runtime_error(fmt::format(
          "Unable to read the value of the <{}> tag.", element_name));
    }

    if (!plant->HasFrameNamed(frame_name)) {
      throw std::runtime_error(fmt::format(
          "Frame: {} specified for <{}> does not exist in the model.",
          frame_name, element_name));
    }

    return plant->GetFrameByName(frame_name);
  };

  return ParseLinearBushingRollPitchYaw(read_vector, read_frame, plant);
}

// Helper to determine if two links are welded together.
bool AreWelded(
    const MultibodyPlant<double>& plant, const Body<double>& a,
    const Body<double>& b) {
  for (auto* body : plant.GetBodiesWeldedTo(a)) {
    if (body == &b) {
      return true;
    }
  }
  return false;
}

// Helper method to add a model to a MultibodyPlant given an sdf::Model
// specification object.
ModelInstanceIndex AddModelFromSpecification(
    const sdf::Model& model,
    const std::string& model_name,
    MultibodyPlant<double>* plant,
    const PackageMap& package_map,
    const std::string& root_dir) {
  const ModelInstanceIndex model_instance =
    plant->AddModelInstance(model_name);

  // TODO(eric.cousineau): Ensure this generalizes to cases when the parent
  // frame is not the world. At present, we assume the parent frame is the
  // world.
  ThrowIfPoseFrameSpecified(model.Element());
  const RigidTransformd X_WM = ToRigidTransform(model.RawPose());

  drake::log()->trace("sdf_parser: Add links");
  std::vector<LinkInfo> added_link_infos = AddLinksFromSpecification(
      model_instance, model, X_WM, plant, package_map, root_dir);

  drake::log()->trace("sdf_parser: Resolve canonical link");
  std::string canonical_link_name = model.CanonicalLinkName();
  if (canonical_link_name.empty()) {
    // TODO(eric.cousineau): Should libsdformat auto-resolve this?
    DRAKE_DEMAND(model.LinkCount() > 0);
    canonical_link_name = model.LinkByIndex(0)->Name();
  }
  const Frame<double>& canonical_link_frame = plant->GetFrameByName(
      canonical_link_name, model_instance);
  const RigidTransformd X_MLc = ResolveRigidTransform(
      model.LinkByName(canonical_link_name)->SemanticPose());

  // Add the SDF "model frame" given the model name so that way any frames added
  // to the plant are associated with this current model instance.
  // N.B. This follows SDFormat's convention.
  const std::string sdf_model_frame_name = "__model__";
  const Frame<double>& model_frame =
      plant->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
          sdf_model_frame_name, canonical_link_frame, X_MLc.inverse(),
          model_instance));

  drake::log()->trace("sdf_parser: Add joints");
  // Add all the joints
  std::set<sdf::JointType> joint_types;
  // TODO(eric.cousineau): Register frames from SDF once we have a pose graph.
  for (uint64_t joint_index = 0; joint_index < model.JointCount();
       ++joint_index) {
    // Get a pointer to the SDF joint, and the joint axis information.
    const sdf::Joint& joint = *model.JointByIndex(joint_index);
    AddJointFromSpecification(
        model, X_WM, joint, model_instance, plant, &joint_types);
  }

  drake::log()->trace("sdf_parser: Add explicit frames");
  // Add frames at root-level of <model>.
  for (uint64_t frame_index = 0; frame_index < model.FrameCount();
       ++frame_index) {
    const sdf::Frame& frame = *model.FrameByIndex(frame_index);
    AddFrameFromSpecification(frame, model_instance, model_frame, plant);
  }

  drake::log()->trace("sdf_parser: Add linear_bushing_rpy");
  if (model.Element()->HasElement("drake:linear_bushing_rpy")) {
    for (sdf::ElementPtr bushing_node =
             model.Element()->GetElement("drake:linear_bushing_rpy");
         bushing_node; bushing_node = bushing_node->GetNextElement(
                           "drake:linear_bushing_rpy")) {
      AddBushingFromSpecification(bushing_node, plant);
    }
  }

  if (model.Static()) {
    // Only weld / fixed joints are permissible.
    // TODO(eric.cousineau): Consider "freezing" non-weld joints, as is
    // permissible in Bullet and DART via Gazebo (#12227).
    for (sdf::JointType joint_type : joint_types) {
      if (joint_type != sdf::JointType::FIXED) {
        throw std::runtime_error(
            "Only fixed joints are permitted in static models.");
      }
    }
    // Weld all links that have been added, but are not (yet) attached to the
    // world.
    // N.B. This implementation prevents "reposturing" a static model after
    // parsing. See #12227 for a more concrete example.
    for (const LinkInfo& link_info : added_link_infos) {
      if (!AreWelded(*plant, plant->world_body(), *link_info.body)) {
        plant->WeldFrames(
            plant->world_frame(), link_info.body->body_frame(), link_info.X_WL);
      }
    }
  }

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
      model, model_name, plant, package_map, root_dir);
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
            model, model.Name(), plant, package_map, root_dir));
    }
  } else {
    // Load the world and all the models in the world.
    const sdf::World& world = *root.WorldByIndex(0);

    // TODO(eric.cousineau): Either support or explicitly prevent adding joints
    // via `//world/joint`, per this Bitbucket comment: https://bit.ly/2udQxhp

    for (uint64_t frame_index = 0; frame_index < world.FrameCount();
        ++frame_index) {
      const sdf::Frame& frame = *world.FrameByIndex(frame_index);
      AddFrameFromSpecification(
          frame, world_model_instance(), plant->world_frame(), plant);
    }

    for (uint64_t model_index = 0; model_index < world.ModelCount();
        ++model_index) {
      // Get the model.
      const sdf::Model& model = *world.ModelByIndex(model_index);
      model_instances.push_back(AddModelFromSpecification(
            model, model.Name(), plant, package_map, root_dir));
    }
  }

  return model_instances;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
