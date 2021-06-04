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
#include "drake/multibody/parsing/detail_ignition.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/parsing/detail_scene_graph.h"
#include "drake/multibody/parsing/scoped_names.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/fixed_offset_frame.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/revolute_spring.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/tree/universal_joint.h"
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

// Given a @p relative_nested_name to an object, this function returns the model
// instance that is an immediate parent of the object and the local name of the
// object. The local name of the object does not have any scoping delimiters.
//
// @param[in] relative_nested_name
//   The name of the object in the scope of the model associated with the given
//   @p model_instance. The relative nested name can contain the scope delimiter
//   to reference objects nested in child models.
// @param[in] model_instance
//   The model instance in whose scope @p relative_nested_name is defined.
// @param[in] plant
//   The MultibodyPlant object that contains the model instance identified by @p
//   model_instance.
// @returns A pair containing the resolved model instance and the local name of
// the object referenced by @p relative_nested_name.
std::pair<ModelInstanceIndex, std::string> GetResolvedModelInstanceAndLocalName(
    const std::string& relative_nested_name, ModelInstanceIndex model_instance,
    const MultibodyPlant<double>& plant) {
  auto [parent_name, unscoped_local_name] =
      sdf::SplitName(relative_nested_name);
  ModelInstanceIndex resolved_model_instance = model_instance;

  if (!parent_name.empty()) {
    // If the parent is in the world_model_instance, the name can be looked up
    // from the plant without creating an absolute name.
    if (world_model_instance() == model_instance) {
      resolved_model_instance = plant.GetModelInstanceByName(parent_name);
    } else {
      const std::string parent_model_absolute_name = sdf::JoinName(
          plant.GetModelInstanceName(model_instance), parent_name);

      resolved_model_instance =
        plant.GetModelInstanceByName(parent_model_absolute_name);
    }
  }

  return {resolved_model_instance, unscoped_local_name};
}

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
          "in <inertial/> or top-level <model/> tags in model files.");
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

std::string ResolveJointParentLinkName(const sdf::Joint& joint) {
  std::string link;
  ThrowAnyErrors(joint.ResolveParentLink(link));
  return link;
}
std::string ResolveJointChildLinkName(const sdf::Joint& joint) {
  std::string link;
  ThrowAnyErrors(joint.ResolveChildLink(link));
  return link;
}

// Helper method to extract the SpatialInertia M_BBo_B of body B, about its body
// frame origin Bo and, expressed in body frame B, from an ignition::Inertial
// object.
SpatialInertia<double> ExtractSpatialInertiaAboutBoExpressedInB(
    const ignition::math::Inertiald& Inertial_BBcm_Bi) {
  double mass = Inertial_BBcm_Bi.MassMatrix().Mass();

  const RotationalInertia<double> I_BBcm_Bi =
      ExtractRotationalInertiaAboutBcmExpressedInBi(Inertial_BBcm_Bi);

  // If this is a massless body, return a zero SpatialInertia.
  if (mass == 0. && I_BBcm_Bi.get_moments().isZero() &&
      I_BBcm_Bi.get_products().isZero()) {
    return SpatialInertia<double>(mass, {0., 0., 0.}, {0., 0., 0});
  }

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
    const std::string& link_name,
    ModelInstanceIndex model_instance, const MultibodyPlant<double>& plant) {
  // SDF's convention to indicate a joint is connected to the world is to either
  // name the corresponding link "world" or just leave it unnamed.
  // Thus this the "if" statement in the following line.
  if (link_name.empty() || link_name == "world") {
    return plant.world_body();
  } else {
    const auto [parent_model_instance, local_name] =
        GetResolvedModelInstanceAndLocalName(link_name, model_instance, plant);

    return plant.GetBodyByName(local_name, parent_model_instance);
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
      joint_spec.Type() == sdf::JointType::PRISMATIC ||
      joint_spec.Type() == sdf::JointType::UNIVERSAL ||
      joint_spec.Type() == sdf::JointType::BALL);

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
    const JointActuator<double>& actuator =
        plant->AddJointActuator(joint_spec.Name(), joint, effort_limit);

    // Parse and add the optional drake:rotor_inertia parameter.
    if (joint_spec.Element()->HasElement("drake:rotor_inertia")) {
      plant->get_mutable_joint_actuator(actuator.index())
          .set_default_rotor_inertia(
              joint_spec.Element()->Get<double>("drake:rotor_inertia"));
    }

    // Parse and add the optional drake:gear_ratio parameter.
    if (joint_spec.Element()->HasElement("drake:gear_ratio")) {
      plant->get_mutable_joint_actuator(actuator.index())
          .set_default_gear_ratio(
              joint_spec.Element()->Get<double>("drake:gear_ratio"));
    }
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
      ResolveJointParentLinkName(joint_spec), model_instance, *plant);
  const Body<double>& child_body = GetBodyByLinkSpecificationName(
      ResolveJointChildLinkName(joint_spec), model_instance, *plant);

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
    case sdf::JointType::UNIVERSAL: {
      const double damping = ParseJointDamping(joint_spec);
      plant->AddJoint<UniversalJoint>(
        joint_spec.Name(),
        parent_body, X_PJ,
        child_body, X_CJ, damping);
      break;
    }
    case sdf::JointType::BALL: {
      const double damping = ParseJointDamping(joint_spec);
      plant->AddJoint<BallRpyJoint>(
        joint_spec.Name(),
        parent_body, X_PJ,
        child_body, X_CJ, damping);
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
    const DataSource& data_source) {
  data_source.DemandExactlyOne();

  std::string root_dir;
  // TODO(marcoag) ensure that we propagate the right ParserConfig instance.
  sdf::ParserConfig parser_config;
  parser_config.SetWarningsPolicy(sdf::EnforcementPolicy::ERR);
  parser_config.SetDeprecatedElementsPolicy(sdf::EnforcementPolicy::WARN);
  // TODO(#15018): This means that unrecognized elements won't be shown to a
  // user directly (via console or exception). We should change unrecognized
  // elements policy to print a warning, and later become an error.
  DRAKE_DEMAND(
      parser_config.UnrecognizedElementsPolicy()
      == sdf::EnforcementPolicy::LOG);

  if (data_source.file_name) {
    const std::string full_path = GetFullPath(*data_source.file_name);
    ThrowAnyErrors(root->Load(full_path, parser_config));
    // Uses the directory holding the SDF to be the root directory
    // in which to search for files referenced within the SDF file.
    size_t found = full_path.find_last_of("/\\");
    if (found != std::string::npos) {
      root_dir = full_path.substr(0, found);
    } else {
      // TODO(jwnimmer-tri) This is not unit tested.  In any case, we should be
      // using drake::filesystem for path manipulation, not string searching.
      root_dir = ".";
    }
  } else {
    DRAKE_DEMAND(data_source.file_contents);
    ThrowAnyErrors(root->LoadSdfString(*data_source.file_contents,
                                       parser_config));
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
  const RigidTransformd X_PF = ResolveRigidTransform(
      frame_spec.SemanticPose(), frame_spec.AttachedTo());
  if (frame_spec.AttachedTo().empty()) {
    parent_frame = &default_frame;
  } else {
    const std::string attached_to_absolute_name = parsing::PrefixName(
        parsing::GetInstanceScopeName(*plant, model_instance),
        frame_spec.AttachedTo());

    // If the attached_to refers to a model, we use the `__model__` frame
    // associated with the model.
    if (plant->HasModelInstanceNamed(attached_to_absolute_name)) {
      const auto attached_to_model_instance =
          plant->GetModelInstanceByName(attached_to_absolute_name);
      parent_frame =
          &plant->GetFrameByName("__model__", attached_to_model_instance);
    } else {
      const auto [parent_model_instance, local_name] =
          GetResolvedModelInstanceAndLocalName(frame_spec.AttachedTo(),
                                               model_instance, *plant);
      parent_frame = &plant->GetFrameByName(local_name, parent_model_instance);
    }
  }
  const Frame<double>& frame =
      plant->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
          frame_spec.Name(), *parent_frame, X_PF));
  return frame;
}

Eigen::Vector3d ParseVector3(const sdf::ElementPtr node,
                             const char* element_name) {
  if (!node->HasElement(element_name)) {
    throw std::runtime_error(
        fmt::format("<{}>: Unable to find the <{}> child tag.", node->GetName(),
                    element_name));
  }

  auto value = node->Get<ignition::math::Vector3d>(element_name);

  return ToVector3(value);
}

const Frame<double>& ParseFrame(const sdf::ElementPtr node,
                                MultibodyPlant<double>* plant,
                                const char* element_name) {
  if (!node->HasElement(element_name)) {
    throw std::runtime_error(
        fmt::format("<{}>: Unable to find the <{}> child tag.", node->GetName(),
                    element_name));
  }

  const std::string frame_name = node->Get<std::string>(element_name);

  if (!plant->HasFrameNamed(frame_name)) {
    throw std::runtime_error(fmt::format(
        "<{}>: Frame '{}' specified for <{}> does not exist in the model.",
        node->GetName(), frame_name, element_name));
  }

  return plant->GetFrameByName(frame_name);
}

// TODO(eric.cousineau): Update parsing pending resolution of
// https://github.com/osrf/sdformat/issues/288
void AddDrakeJointFromSpecification(const sdf::ElementPtr node,
                                    MultibodyPlant<double>* plant) {
  if (!node->HasAttribute("type")) {
    throw std::runtime_error(
        "<drake:joint>: Unable to find the 'type' attribute.");
  }
  const std::string joint_type = node->Get<std::string>("type");
  if (!node->HasAttribute("name")) {
    throw std::runtime_error(
        "<drake:joint>: Unable to find the 'name' attribute.");
  }
  const std::string joint_name = node->Get<std::string>("name");

  // TODO(eric.cousineau): Add support for parsing joint pose.
  if (node->HasElement("pose")) {
    throw std::runtime_error(
        "<drake:joint> does not yet support the <pose> child tag.");
  }

  const Frame<double>& parent_frame = ParseFrame(node, plant, "drake:parent");
  const Frame<double>& child_frame = ParseFrame(node, plant, "drake:child");

  if (joint_type == "planar") {
    // TODO(eric.cousineau): Error out when there are unused tags.
    Vector3d damping = ParseVector3(node, "drake:damping");
    plant->AddJoint(std::make_unique<PlanarJoint<double>>(
        joint_name, parent_frame, child_frame, damping));
  } else {
    throw std::runtime_error(
        "ERROR: <drake:joint> '" + joint_name +
        "' has unrecognized value for 'type' attribute: " + joint_type);
  }
}

const LinearBushingRollPitchYaw<double>& AddBushingFromSpecification(
    const sdf::ElementPtr node, MultibodyPlant<double>* plant) {
  // Functor to read a vector valued child tag with tag name: `element_name`
  // e.g. <element_name>0 0 0</element_name>
  // Throws an error if the tag does not exist.
  auto read_vector = [node](const char* element_name) -> Eigen::Vector3d {
    return ParseVector3(node, element_name);
  };

  // Functor to read a child tag with tag name: `element_name` that specifies a
  // frame name, e.g. <element_name>frame_name</element_name>
  // Throws an error if the tag does not exist or if the frame does not exist in
  // the plant.
  auto read_frame = [node,
                     plant](const char* element_name) -> const Frame<double>& {
    return ParseFrame(node, plant, element_name);
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
std::vector<ModelInstanceIndex> AddModelsFromSpecification(
    const sdf::Model& model,
    const std::string& model_name,
    const RigidTransformd& X_WP,
    MultibodyPlant<double>* plant,
    const PackageMap& package_map,
    const std::string& root_dir) {
  const ModelInstanceIndex model_instance =
    plant->AddModelInstance(model_name);
  std::vector <ModelInstanceIndex> added_model_instances{model_instance};

  // "P" is the parent frame. If the model is in a child of //world or //sdf,
  // this will be the world frame. Otherwise, this will be the parent model
  // frame.
  const RigidTransformd X_PM = ResolveRigidTransform(model.SemanticPose());
  const RigidTransformd X_WM = X_WP * X_PM;

  // Add nested models at root-level of <model>.
  // Do this before the resolving canonical link because the link might be in a
  // nested model.
  drake::log()->trace("sdf_parser: Add nested models");
  for (uint64_t model_index = 0; model_index < model.ModelCount();
       ++model_index) {
    const sdf::Model& nested_model = *model.ModelByIndex(model_index);
    std::vector<ModelInstanceIndex> nested_model_instances =
        AddModelsFromSpecification(
            nested_model, sdf::JoinName(model_name, nested_model.Name()), X_WM,
            plant, package_map, root_dir);

    DRAKE_DEMAND(!nested_model_instances.empty());
    const ModelInstanceIndex nested_model_instance =
        nested_model_instances.front();

    plant->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
        nested_model.Name(),
        plant->GetFrameByName("__model__", nested_model_instance),
        RigidTransformd::Identity(), model_instance));

    added_model_instances.insert(added_model_instances.end(),
                                 nested_model_instances.begin(),
                                 nested_model_instances.end());
  }

  drake::log()->trace("sdf_parser: Add links");
  std::vector<LinkInfo> added_link_infos = AddLinksFromSpecification(
      model_instance, model, X_WM, plant, package_map, root_dir);

  // Add the SDF "model frame" given the model name so that way any frames added
  // to the plant are associated with this current model instance.
  // N.B. This follows SDFormat's convention.
  const std::string sdf_model_frame_name = "__model__";

  drake::log()->trace("sdf_parser: Resolve canonical link");
  const Frame<double>& model_frame = [&]() -> const Frame<double>& {
    const auto [canonical_link, canonical_link_name] =
        model.CanonicalLinkAndRelativeName();

    if (canonical_link != nullptr) {
      const auto [parent_model_instance, local_name] =
          GetResolvedModelInstanceAndLocalName(canonical_link_name,
                                             model_instance, *plant);
      const Frame<double>& canonical_link_frame =
          plant->GetFrameByName(local_name, parent_model_instance);
      const RigidTransformd X_LcM = ResolveRigidTransform(
          model.SemanticPose(),
          sdf::JoinName(model.Name(), canonical_link_name));

      return plant->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
          sdf_model_frame_name, canonical_link_frame, X_LcM, model_instance));
    } else {
      return plant->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
          sdf_model_frame_name, plant->world_frame(), X_WM, model_instance));
    }
  }();

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

  drake::log()->trace("sdf_parser: Add drake custom joints");
  if (model.Element()->HasElement("drake:joint")) {
    for (sdf::ElementPtr joint_node =
             model.Element()->GetElement("drake:joint");
         joint_node; joint_node = joint_node->GetNextElement("drake:joint")) {
      AddDrakeJointFromSpecification(joint_node, plant);
    }
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
    // N.B. This implementation complicates "reposturing" a static model after
    // parsing. See #12227 and #14518 for more discussion.
    for (const LinkInfo& link_info : added_link_infos) {
      if (!AreWelded(*plant, plant->world_body(), *link_info.body)) {
        const auto& A = plant->world_frame();
        const auto& B = link_info.body->body_frame();
        const std::string joint_name =
            "sdformat_model_static_" + A.name() + "_welds_to_" + B.name();
        plant->AddJoint(
            std::make_unique<WeldJoint<double>>(
                joint_name, A, B, link_info.X_WL));
      }
    }
  }

  return added_model_instances;
}

}  // namespace

ModelInstanceIndex AddModelFromSdf(
    const DataSource& data_source,
    const std::string& model_name_in,
    const PackageMap& package_map,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(!plant->is_finalized());

  sdf::Root root;

  std::string root_dir = LoadSdf(&root, data_source);

  // TODO(jwnimmer-tri) When we upgrade to a version of libsdformat that no
  // longer offers ModelCount(), remove this entire paragraph of code.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const uint64_t model_count = root.ModelCount();
#pragma GCC diagnostic pop
  if (model_count != 1) {
    throw std::runtime_error("File must have a single <model> element.");
  }

  // Get the only model in the file.
  const sdf::Model& model = *root.Model();

  // //sdf/model/pose/@relative_to is invalid. Note, libsdformat should emit an
  // error during Load, but currently doesn't. See sdformat#567
  ThrowIfPoseFrameSpecified(model.Element());

  if (scene_graph != nullptr && !plant->geometry_source_is_registered()) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
  }

  const std::string model_name =
      model_name_in.empty() ? model.Name() : model_name_in;

  std::vector<ModelInstanceIndex> added_model_instances =
      AddModelsFromSpecification(model, model_name, {}, plant, package_map,
                                 root_dir);

  DRAKE_DEMAND(!added_model_instances.empty());
  return added_model_instances.front();
}

std::vector<ModelInstanceIndex> AddModelsFromSdf(
    const DataSource& data_source,
    const PackageMap& package_map,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(!plant->is_finalized());

  sdf::Root root;

  std::string root_dir = LoadSdf(&root, data_source);

  // Throw an error if there are no models or worlds.
  if (root.Model() == nullptr && root.WorldCount() == 0) {
    throw std::runtime_error(
        "File must have at least one <model>, or <world> with one "
        "child <model> element.");
  }

  // Only one world in an SDF file is allowed.
  if (root.WorldCount() > 1) {
    throw std::runtime_error("File must contain only one <world>.");
  }

  // Do not allow a <world> to have a <model> sibling.
  if (root.Model() != nullptr && root.WorldCount() > 0) {
    throw std::runtime_error("A <world> and <model> cannot be siblings.");
  }

  if (scene_graph != nullptr && !plant->geometry_source_is_registered()) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
  }

  std::vector<ModelInstanceIndex> model_instances;

  // At this point there should be only Models or a single World at the Root
  // level.
  if (root.Model() != nullptr) {
    // Load all the models at the root level.
    // TODO(jwnimmer-tri) When we upgrade to a version of libsdformat that no
    // longer offers ModelCount(), we should simplify this entire block by
    // removing the for-each-model loop, instead just using the root.Model().
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    const uint64_t model_count = root.ModelCount();
#pragma GCC diagnostic pop
    if (model_count != 1) {
      static const logging::Warn log_once(
        "The feature to load multiple models from a single SDFormat model file "
        "in Drake is deprecated, and will be removed on or around 2021-08-01. "
        "If you need multiple root-level models, please use an SDFormat world "
        "file.");
    }
    for (uint64_t i = 0; i < model_count; ++i) {
      // Get the model.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      const sdf::Model& model = *root.ModelByIndex(i);
#pragma GCC diagnostic pop
      // //sdf/model/pose/@relative_to is invalid. Note, libsdformat should emit
      // an error during Load, but currently doesn't. See sdformat#567
      ThrowIfPoseFrameSpecified(model.Element());

      std::vector<ModelInstanceIndex> added_model_instances =
          AddModelsFromSpecification(model, model.Name(), {}, plant,
                                     package_map, root_dir);
      model_instances.insert(model_instances.end(),
                             added_model_instances.begin(),
                             added_model_instances.end());
    }
  } else {
    // Load the world and all the models in the world.
    const sdf::World& world = *root.WorldByIndex(0);

    // TODO(eric.cousineau): Either support or explicitly prevent adding joints
    // via `//world/joint`, per this Bitbucket comment: https://bit.ly/2udQxhp

    for (uint64_t model_index = 0; model_index < world.ModelCount();
        ++model_index) {
      // Get the model.
      const sdf::Model& model = *world.ModelByIndex(model_index);
      std::vector<ModelInstanceIndex> added_model_instances =
          AddModelsFromSpecification(model, model.Name(), {}, plant,
                                     package_map, root_dir);
      model_instances.insert(model_instances.end(),
                             added_model_instances.begin(),
                             added_model_instances.end());
    }

    for (uint64_t frame_index = 0; frame_index < world.FrameCount();
        ++frame_index) {
      const sdf::Frame& frame = *world.FrameByIndex(frame_index);
      AddFrameFromSpecification(
          frame, world_model_instance(), plant->world_frame(), plant);
    }
  }

  return model_instances;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
