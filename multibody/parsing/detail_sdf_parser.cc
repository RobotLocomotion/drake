#include "drake/multibody/parsing/detail_sdf_parser.h"

#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include <sdf/Error.hh>
#include <sdf/Frame.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/ParserConfig.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_constant_curvature_trajectory.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/detail_ignition.h"
#include "drake/multibody/parsing/detail_make_model_name.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/parsing/detail_sdf_diagnostic.h"
#include "drake/multibody/parsing/detail_sdf_geometry.h"
#include "drake/multibody/parsing/detail_urdf_parser.h"
#include "drake/multibody/parsing/scoped_names.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/curvilinear_joint.h"
#include "drake/multibody/tree/fixed_offset_frame.h"
#include "drake/multibody/tree/geometry_spatial_inertia.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/prismatic_spring.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/revolute_spring.h"
#include "drake/multibody/tree/scoped_name.h"
#include "drake/multibody/tree/screw_joint.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/tree/universal_joint.h"
#include "drake/multibody/tree/weld_joint.h"

namespace drake {
namespace multibody {
namespace internal {

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;
using Eigen::Matrix3d;
using Eigen::Translation3d;
using Eigen::Vector3d;
using geometry::GeometryInstance;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;
using std::unique_ptr;

// Unnamed namespace for free functions local to this file.
namespace {

// A short summary of parsing stages in libsdformat:
//
// When we load a model using `sdf::Root::Load`, the following happens:
// * libsdformat parses the XML into `sdf::ElementPtr` objects. Included
// SDFormat models are expanded during this process. If the included model is
// not an SDFormat file type and there are custom parsers registered in the
// passed in `ParserConfig`, the contents of the `<include>` tag are saved for
// later processing
// * DOM objects are created from `sdf::ElementPtr` objects. During the creation
// of `sdf::World` and `sdf::Model`, custom parsers are called on the
// non-SDFormat included files mentioned above. This is what's called the
// Interface API.
// * Drake's custom parser (eg. detail_urdf_parser) parses the files into a
// MultibodyPlant.
// * The frame bearing elements (Links, Joints, Frames, and Models) are read
// back from the MultibodyPlant and returned to libsdformat via the Interface
// API data structures.
// * Once DOM object creation is complete, libsdformat constructs the frame and
// pose graph for the entire model.
// * `sdf::Root::Load` returns and Drake starts its parsing stage where it
// populates the MultibodyPlant using the DOM objects created above (e.g.
// AddModelsFromSpecification).

// `ModelInstanceIndexRange` is a data structure to hold model instances that
// were created during libsdformat's Interface API callback for handling merged
// models. When handling merged models during libsdformat's Interface API
// callback, we have to create model instances of parents of merged models since
// the parent models are regular SDFormat elements and will only be handled at a
// later stage of parsing. This data structure is used to store these parent
// model instances so that when we're going through Drake's parsing stage (last
// step in the parsing stage summary above), we are aware of these model
// instances and not try to recreate them.
using ModelInstanceIndexRange =
    std::pair<ModelInstanceIndex, ModelInstanceIndex>;

// Returns the model instance name for the given `instance`, unless it's the
// world model instance in which case returns the empty string.
std::string GetInstanceScopeNameIgnoringWorld(
    const MultibodyPlant<double>& plant, ModelInstanceIndex instance) {
  if (instance != plant.world_body().model_instance()) {
    return plant.GetModelInstanceName(instance);
  } else {
    return "";
  }
}

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

// Calculates the scoped name of a body relative to the model instance @p
// relative_to_model_instance. If the body is a direct child of the model,
// this simply returns the local name of the body. However, if the body is
// a child of a nested model, the local name of the body is prefixed with the
// scoped name of the nested model. If the relative_to_model_instance is the
// world_model_instance, the local name of the body is prefixed with the model
// name.
std::string GetRelativeBodyName(const RigidBody<double>& body,
                                ModelInstanceIndex relative_to_model_instance,
                                const MultibodyPlant<double>& plant) {
  const std::string& relative_to_model_absolute_name =
      plant.GetModelInstanceName(relative_to_model_instance);
  // If the relative_to_model instance is the world_model_instance, we need to
  // prefix the body name with the model name
  if (relative_to_model_instance == world_model_instance()) {
    const std::string& model_absolute_name =
        plant.GetModelInstanceName(body.model_instance());

    return sdf::JoinName(model_absolute_name, body.name());
  } else if (body.model_instance() != relative_to_model_instance) {
    // If the body is inside a nested model, we need to prefix the
    // name with the relative name of the nested model.
    const std::string& nested_model_absolute_name =
        plant.GetModelInstanceName(body.model_instance());
    // The relative_to_model_absolute_name must be a prefix of the
    // nested_model_absolute_name. Otherwise the nested model is not a
    // descendent of the model relative to which we are computing the name.
    const std::string required_prefix =
        relative_to_model_absolute_name + std::string(sdf::kScopeDelimiter);
    DRAKE_DEMAND(nested_model_absolute_name.starts_with(required_prefix));

    const std::string nested_model_relative_name =
        nested_model_absolute_name.substr(required_prefix.size());

    return sdf::JoinName(nested_model_relative_name, body.name());
  } else {
    return body.name();
  }
}

// This takes an `sdf::SemanticPose`, which defines a pose relative to a frame,
// and resolves its value with respect to another frame.
math::RigidTransformd ResolveRigidTransform(
    const SDFormatDiagnostic& diagnostic,
    const sdf::SemanticPose& semantic_pose,
    const std::string& relative_to = "") {
  gz::math::Pose3d pose;
  sdf::Errors errors = semantic_pose.Resolve(pose, relative_to);
  diagnostic.PropagateErrors(errors);
  return ToRigidTransform(pose);
}

Eigen::Vector3d ResolveAxisXyz(const SDFormatDiagnostic& diagnostic,
                               const sdf::JointAxis& axis) {
  gz::math::Vector3d xyz;
  sdf::Errors errors = axis.ResolveXyz(xyz);
  diagnostic.PropagateErrors(errors);
  return ToVector3(xyz);
}

std::string ResolveJointParentLinkName(const SDFormatDiagnostic& diagnostic,
                                       const sdf::Joint& joint) {
  std::string link;
  sdf::Errors errors = joint.ResolveParentLink(link);
  diagnostic.PropagateErrors(errors);
  return link;
}

std::string ResolveJointChildLinkName(const SDFormatDiagnostic& diagnostic,
                                      const sdf::Joint& joint) {
  std::string link;
  sdf::Errors errors = joint.ResolveChildLink(link);
  diagnostic.PropagateErrors(errors);
  return link;
}

// Helper method to extract the SpatialInertia M_BBo_B of body B, about its body
// frame origin Bo and, expressed in body frame B, from a gz::Inertial
// object.
SpatialInertia<double> ExtractSpatialInertiaAboutBoExpressedInB(
    const SDFormatDiagnostic& diagnostic, const sdf::ElementPtr link_element,
    const gz::math::Inertiald& Inertial_BBcm_Bi) {
  double mass = Inertial_BBcm_Bi.MassMatrix().Mass();

  // Pose of the "<inertial>" frame Bi in the body frame B.
  // TODO(amcastro-tri): Verify we don't get funny results when X_BBi is not
  // the identity matrix.
  // TODO(amcastro-tri): SDF seems to provide <inertial><frame/></inertial>
  // together with <inertial><pose/></inertial>. It'd seem then the frame B
  // in X_BI could be another frame. We rely on Inertial::Pose() to ALWAYS
  // give us X_BI. Verify this.
  const RigidTransformd X_BBi = ToRigidTransform(Inertial_BBcm_Bi.Pose());

  // TODO(amcastro-tri): Verify that gz::math::Inertial::MOI() ALWAYS is
  // expresed in the body frame B, regardless of how a user might have
  // specified frames in the sdf file. That is, that it always returns R_BBcm_B.
  // TODO(amcastro-tri): Verify that gz::math::Inertial::MassMatrix()
  // ALWAYS is in the inertial frame Bi, regardless of how a user might have
  // specified frames in the sdf file. That is, that it always returns
  // M_BBcm_Bi.
  const gz::math::Matrix3d I = Inertial_BBcm_Bi.MassMatrix().Moi();
  return ParseSpatialInertia(diagnostic.MakePolicyForNode(*link_element), X_BBi,
                             mass,
                             {.ixx = I(0, 0),
                              .iyy = I(1, 1),
                              .izz = I(2, 2),
                              .ixy = I(1, 0),
                              .ixz = I(2, 0),
                              .iyz = I(2, 1)});
}

// Helper method to retrieve a Body given the name of the link specification.
const RigidBody<double>& GetBodyByLinkSpecificationName(
    const std::string& link_name, ModelInstanceIndex model_instance,
    const MultibodyPlant<double>& plant) {
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
Vector3d ExtractJointAxis(const SDFormatDiagnostic& diagnostic,
                          const sdf::Joint& joint_spec) {
  DRAKE_DEMAND(joint_spec.Type() == sdf::JointType::REVOLUTE ||
               joint_spec.Type() == sdf::JointType::SCREW ||
               joint_spec.Type() == sdf::JointType::PRISMATIC ||
               joint_spec.Type() == sdf::JointType::CONTINUOUS);

  // Axis specification.
  const sdf::JointAxis* axis = joint_spec.Axis();
  if (axis == nullptr) {
    std::string message = fmt::format(
        "An axis must be specified for joint '{}'", joint_spec.Name());
    diagnostic.Error(joint_spec.Element(), std::move(message));
    return Vector3d(0, 0, 1);
  }

  // Joint axis in the joint frame J.
  Vector3d axis_J = ResolveAxisXyz(diagnostic, *axis);
  return axis_J;
}

// Extracts a Vector3d representation of `axis` and `axis2` for joints with both
// attributes. Both axes are required. Otherwise, an error is triggered.
std::pair<Vector3d, Vector3d> ExtractJointAxisAndAxis2(
    const SDFormatDiagnostic& diagnostic, const sdf::Joint& joint_spec) {
  DRAKE_DEMAND(joint_spec.Type() == sdf::JointType::REVOLUTE2 ||
               joint_spec.Type() == sdf::JointType::UNIVERSAL);

  // Axis specification.
  const sdf::JointAxis* axis = joint_spec.Axis(0);
  const sdf::JointAxis* axis2 = joint_spec.Axis(1);
  if (axis == nullptr || axis2 == nullptr) {
    std::string message =
        fmt::format("Both axis and axis2 must be specified for joint '{}'",
                    joint_spec.Name());
    diagnostic.Error(joint_spec.Element(), std::move(message));
    return std::make_pair(Vector3d(1, 0, 0), Vector3d(0, 1, 0));
  }

  // Joint axis and axis2 in the joint frame J.
  Vector3d axis_J = ResolveAxisXyz(diagnostic, *axis);
  Vector3d axis2_J = ResolveAxisXyz(diagnostic, *axis2);
  return std::make_pair(axis_J, axis2_J);
}

// Helper to parse the damping for a given joint specification.
// Right now we only parse the <damping> tag.
double ParseJointDamping(const SDFormatDiagnostic& diagnostic,
                         const sdf::Joint& joint_spec) {
  DRAKE_DEMAND(joint_spec.Type() == sdf::JointType::REVOLUTE ||
               joint_spec.Type() == sdf::JointType::PRISMATIC ||
               joint_spec.Type() == sdf::JointType::SCREW ||
               joint_spec.Type() == sdf::JointType::UNIVERSAL ||
               joint_spec.Type() == sdf::JointType::BALL ||
               joint_spec.Type() == sdf::JointType::CONTINUOUS);

  // If the axis is missing, we'll rely on ExtractJointAxis to tell the user.
  // For our purposes in this function, it's OK to just bail and return zero.
  const sdf::JointAxis* axis = joint_spec.Axis();
  if (axis == nullptr) {
    return 0.0;
  }
  const double damping = axis->Damping();
  if (damping < 0) {
    std::string message = fmt::format(
        "Joint damping is negative for joint '{}'. "
        "Joint damping must be a non-negative number.",
        joint_spec.Name());
    diagnostic.Error(joint_spec.Element(), std::move(message));
    return 0.0;
  }
  // If there are more than one axis (e.g. universal joint), we ignore the
  // damping specified for the second axis.
  const sdf::JointAxis* axis2 = joint_spec.Axis(1);
  if (axis2 != nullptr) {
    const double damping2 = axis2->Damping();
    if (damping2 != damping) {
      std::string message = fmt::format(
          "Joint damping must be equal for both axes for joint {}. "
          "The damping coefficient for 'axis' ({}) is used. The "
          "value for 'axis2' ({}) is ignored. The damping coefficient "
          "for 'axis2' should be explicitly defined as {} to match that for "
          "'axis'.",
          joint_spec.Name(), damping, damping2, damping);
      diagnostic.Warning(joint_spec.Element(), std::move(message));
    }
  }

  return damping;
}

// We interpret a value of exactly zero to specify un-actuated joints. Thus, the
// user would say <effort>0</effort>. The effort_limit should be non-negative.
// SDFormat internally interprets a negative effort limit as infinite and only
// returns non-negative values.
double GetEffortLimit(const SDFormatDiagnostic& diagnostic,
                      const sdf::Joint& joint_spec, int axis_index) {
  DRAKE_DEMAND(axis_index == 0 || axis_index == 1);
  const sdf::JointAxis* axis = joint_spec.Axis(axis_index);
  if (axis == nullptr) {
    std::string message =
        fmt::format("An axis{} must be specified for joint '{}'",
                    axis_index > 0 ? "2" : "", joint_spec.Name());
    diagnostic.Warning(joint_spec.Element(), std::move(message));
    return 0.0;
  }
  return axis->Effort();
}

// Extracts the effort limit from a joint specification and adds an actuator if
// the value is non-zero. In SDFormat, effort limits are specified in
// <joint><axis><limit><effort>. In Drake, we understand that joints with an
// effort limit of zero are not actuated. For joint types that do not have an
// actuator implementation available in Drake, produces a diagnostic warning.
void AddJointActuatorFromSpecification(const SDFormatDiagnostic& diagnostic,
                                       const sdf::Joint& joint_spec,
                                       const Joint<double>& joint,
                                       MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_DEMAND(joint_spec.Type() == sdf::JointType::BALL ||
               joint_spec.Type() == sdf::JointType::SCREW ||
               joint_spec.Type() == sdf::JointType::UNIVERSAL ||
               joint_spec.Type() == sdf::JointType::PRISMATIC ||
               joint_spec.Type() == sdf::JointType::REVOLUTE ||
               joint_spec.Type() == sdf::JointType::CONTINUOUS);

  // Ball joints do not have an axis (nor actuation). However, Drake still
  // permits the declaration of a first axis in order to specify damping, but
  // it should not have actuation, nor should there be a second axis.
  if (joint_spec.Type() == sdf::JointType::BALL) {
    if (joint_spec.Axis(0) != nullptr) {
      std::string message = fmt::format(
          "A ball joint axis will be ignored. Only the dynamic parameters"
          " and limits will be considered.",
          joint_spec.Name());
      diagnostic.Warning(joint_spec.Element(), std::move(message));
      if (GetEffortLimit(diagnostic, joint_spec, 0) != 0) {
        std::string effort_message = fmt::format(
            "Actuation (via non-zero effort limits) for ball joint '{}' is"
            " not implemented yet and will be ignored.",
            joint_spec.Name());
        diagnostic.Warning(joint_spec.Element(), std::move(effort_message));
      }
    }
    if (joint_spec.Axis(1) != nullptr) {
      std::string message = fmt::format(
          "An axis2 may not be specified for ball joint '{}' and will be "
          "ignored",
          joint_spec.Name());
      diagnostic.Warning(joint_spec.Element(), std::move(message));
    }
    return;
  }

  // Universal joints should have both axes defined per SDFormat, but Drake
  // does not yet implement a 2-dof actuator for this joint type.
  if (joint_spec.Type() == sdf::JointType::UNIVERSAL) {
    if (GetEffortLimit(diagnostic, joint_spec, 0) != 0 ||
        GetEffortLimit(diagnostic, joint_spec, 1) != 0) {
      std::string message = fmt::format(
          "Actuation (via non-zero effort limits) for universal joint '{}' is"
          " not implemented yet and will be ignored.",
          joint_spec.Name());
      diagnostic.Warning(joint_spec.Element(), std::move(message));
    }
    return;
  }

  // Prismatic, screw, revolute, and continuous joints have a single axis.
  const double effort_limit = GetEffortLimit(diagnostic, joint_spec, 0);
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

    // Parse and add the optional drake:controller_gains parameter.
    if (joint_spec.Element()->HasElement("drake:controller_gains")) {
      sdf::ElementPtr controller_gains =
          joint_spec.Element()->GetElement("drake:controller_gains");

      const bool has_p = controller_gains->HasAttribute("p");
      const bool has_d = controller_gains->HasAttribute("d");

      if (!has_p) {
        std::string message =
            "<drake:controller_gains>: Unable to find the 'p' attribute.";
        diagnostic.Error(controller_gains, std::move(message));
      }
      if (!has_d) {
        std::string message =
            "<drake:controller_gains>: Unable to find the 'd' attribute.";
        diagnostic.Error(controller_gains, std::move(message));
      }

      if (has_p && has_d) {
        const double p = controller_gains->Get<double>("p");
        const double d = controller_gains->Get<double>("d");

        plant->get_mutable_joint_actuator(actuator.index())
            .set_controller_gains({p, d});
      }
    }
  }
  if (joint_spec.Axis(1) != nullptr) {
    std::string message = fmt::format(
        "An axis2 may not be specified for 1-dof joint '{}' and will be "
        "ignored",
        joint_spec.Name());
    diagnostic.Warning(joint_spec.Element(), std::move(message));
  }
}

// Extracts the spring stiffness and the spring reference from a joint
// specification and adds a prismatic spring force element with the
// corresponding spring reference if the spring stiffness is nonzero.
// Only available for "prismatic" joints. The units for spring
// reference is meter and the units for spring stiffness is N/m.
void AddPrismaticSpringFromSpecification(const SDFormatDiagnostic& diagnostic,
                                         const sdf::Joint& joint_spec,
                                         const PrismaticJoint<double>& joint,
                                         MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(joint_spec.Type() == sdf::JointType::PRISMATIC);

  // Axis specification.
  const sdf::JointAxis* axis = joint_spec.Axis();
  if (axis == nullptr) {
    std::string message = fmt::format(
        "An axis must be specified for joint '{}'.", joint_spec.Name());
    diagnostic.Error(joint_spec.Element(), std::move(message));
    return;
  }

  const double spring_reference = axis->SpringReference();
  const double spring_stiffness = axis->SpringStiffness();

  // We add a force element if stiffness is positive, report error
  // if the stiffness is negative, and pass if the stiffness is zero
  if (spring_stiffness > 0) {
    plant->AddForceElement<PrismaticSpring>(joint, spring_reference,
                                            spring_stiffness);
  } else if (spring_stiffness < 0) {
    std::string message = fmt::format(
        "The stiffness specified for joint '{}' must be non-negative.",
        joint_spec.Name());
    diagnostic.Error(joint_spec.Element(), std::move(message));
  }
}

// Extracts the spring stiffness and the spring reference from a joint
// specification and adds a revolute spring force element with the
// corresponding spring reference if the spring stiffness is nonzero.
// Only available for "revolute" and "continuous" joints. The units for spring
// reference is radians and the units for spring stiffness is N⋅m/rad.
// When the error diagnostic policy is not set to throw this function will
// return false on errors.
bool AddRevoluteSpringFromSpecification(const SDFormatDiagnostic& diagnostic,
                                        const sdf::Joint& joint_spec,
                                        const RevoluteJoint<double>& joint,
                                        MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(joint_spec.Type() == sdf::JointType::REVOLUTE ||
                     joint_spec.Type() == sdf::JointType::CONTINUOUS);

  // Axis specification.
  const sdf::JointAxis* axis = joint_spec.Axis();
  if (axis == nullptr) {
    std::string message =
        "An axis must be specified for joint '" + joint_spec.Name() + "'";
    diagnostic.Error(joint_spec.Element(), std::move(message));
    return false;
  }

  const double spring_reference = axis->SpringReference();
  const double spring_stiffness = axis->SpringStiffness();

  // We don't add a force element if stiffness is zero.
  // If a negative value is passed in, RevoluteSpring will
  // throw an error.
  if (spring_stiffness != 0) {
    plant->AddForceElement<RevoluteSpring>(joint, spring_reference,
                                           spring_stiffness);
  }

  return true;
}

// Returns joint limits as the tuple (lower_limit, upper_limit,
// velocity_limit, acceleration_limit).  The units of the limits depend on the
// particular joint type.  For prismatic joints, units are meters for the
// position limits and m/s for the velocity limit.  For revolute, units
// are radians for the position limits and rad/s for the velocity limit. For
// continuous, positions limits are infinities and velocity limits have units
// rad/s. Velocity and acceleration limits are always >= 0.  This method throws
// an exception if the joint type is not one of revolute, prismatic, or
// continuous. When the diagnostic policy is not set to throw it will return
// std::nullopt on errors.
std::optional<std::tuple<double, double, double, double>> ParseJointLimits(
    const SDFormatDiagnostic& diagnostic, const sdf::Joint& joint_spec) {
  DRAKE_THROW_UNLESS(joint_spec.Type() == sdf::JointType::REVOLUTE ||
                     joint_spec.Type() == sdf::JointType::PRISMATIC ||
                     joint_spec.Type() == sdf::JointType::CONTINUOUS);
  // Axis specification.
  const sdf::JointAxis* axis = joint_spec.Axis();
  if (axis == nullptr) {
    std::string message =
        "An axis must be specified for joint '" + joint_spec.Name() + "'";
    diagnostic.Error(joint_spec.Element(), std::move(message));
    return std::nullopt;
  }

  // As of libsdformat13, ±∞ are used for axes with no position limits,
  // so no special handling is needed.
  const double lower_limit = axis->Lower();
  const double upper_limit = axis->Upper();
  if (lower_limit > upper_limit) {
    std::string message =
        "The lower limit must be lower (or equal) than "
        "the upper limit for joint '" +
        joint_spec.Name() + "'.";
    diagnostic.Error(joint_spec.Element(), std::move(message));
    return std::nullopt;
  }

  // SDFormat internally interprets a negative velocity limit as infinite and
  // only returns non-negative values.
  const double velocity_limit = axis->MaxVelocity();

  // Read Drake-namespaced acceleration limit if present. If not, default to
  // ±numeric_limits<double>::infinity().
  double acceleration_limit = std::numeric_limits<double>::infinity();
  if (axis->Element()->HasElement("limit")) {
    const auto limit_element = axis->Element()->GetElement("limit");
    const std::set<std::string> supported_limit_elements{
        "drake:acceleration", "effort", "lower",
        "stiffness",          "upper",  "velocity"};
    CheckSupportedElements(diagnostic, limit_element, supported_limit_elements);

    if (limit_element->HasElement("drake:acceleration")) {
      acceleration_limit = limit_element->Get<double>("drake:acceleration");
      if (acceleration_limit < 0) {
        std::string message = "Acceleration limit is negative for joint '" +
                              joint_spec.Name() +
                              "'. Aceleration limit must be a non-negative"
                              " number.";
        diagnostic.Error(limit_element, std::move(message));
        return std::tuple<double, double, double, double>();
      }
    }
  }

  return std::make_tuple(lower_limit, upper_limit, velocity_limit,
                         acceleration_limit);
}

// Helper method to add joints to a MultibodyPlant given an sdf::Joint
// specification object. X_WM should be an identity when adding a world
// joint (is_model_joint = false) since a world joint doesn't have a
// containing model, hence M = W.
// If the diagnostic error policy is not set to throw it returns false
// when an error occurs.
bool AddJointFromSpecification(const SDFormatDiagnostic& diagnostic,
                               const RigidTransformd& X_WM,
                               const sdf::Joint& joint_spec,
                               ModelInstanceIndex model_instance,
                               MultibodyPlant<double>* plant,
                               std::set<sdf::JointType>* joint_types,
                               bool is_model_joint = true) {
  const std::set<std::string> supported_joint_elements{"axis",
                                                       "axis2",
                                                       "child",
                                                       "drake:rotor_inertia",
                                                       "drake:gear_ratio",
                                                       "drake:controller_gains",
                                                       "drake:mimic",
                                                       "parent",
                                                       "pose",
                                                       "screw_thread_pitch"};
  CheckSupportedElements(diagnostic, joint_spec.Element(),
                         supported_joint_elements);

  // Axis elements should be fully supported, let sdformat validate those.

  const RigidBody<double>& parent_body = GetBodyByLinkSpecificationName(
      ResolveJointParentLinkName(diagnostic, joint_spec), model_instance,
      *plant);
  const RigidBody<double>& child_body = GetBodyByLinkSpecificationName(
      ResolveJointChildLinkName(diagnostic, joint_spec), model_instance,
      *plant);

  // Get the pose of frame J in the frame of the child link C, as specified in
  // <joint> <pose> ... </pose></joint>. The default `relative_to` pose of a
  // joint will be the child link.
  const RigidTransformd X_CJ = ResolveRigidTransform(
      diagnostic, joint_spec.SemanticPose(),
      GetRelativeBodyName(child_body, model_instance, *plant));

  // Pose of the frame J in the parent body frame P.
  std::optional<RigidTransformd> X_PJ;
  // We need to treat the world case separately since sdformat does not create
  // a "world" link from which we can request its pose (which in that case would
  // be the identity).
  const std::string relative_to = (is_model_joint) ? "__model__" : "world";
  if (parent_body.index() == world_index()) {
    const RigidTransformd X_MJ = ResolveRigidTransform(
        diagnostic, joint_spec.SemanticPose(), relative_to);
    X_PJ = X_WM * X_MJ;  // Since P == W.
  } else {
    X_PJ = ResolveRigidTransform(
        diagnostic, joint_spec.SemanticPose(),
        GetRelativeBodyName(parent_body, model_instance, *plant));
  }

  // If P and J are coincident, we won't create a new frame for J, but use frame
  // P directly. We indicate that by passing a nullopt.
  if (X_PJ.value().IsExactlyIdentity()) X_PJ = std::nullopt;

  // These will only be populated for prismatic and revolute joints.
  double lower_limit = 0;
  double upper_limit = 0;
  double velocity_limit = 0;
  double acceleration_limit = 0;

  switch (joint_spec.Type()) {
    case sdf::JointType::FIXED: {
      plant->AddJoint<WeldJoint>(joint_spec.Name(), parent_body, X_PJ,
                                 child_body, X_CJ,
                                 RigidTransformd::Identity() /* X_JpJc */);
      break;
    }
    case sdf::JointType::PRISMATIC: {
      const double damping = ParseJointDamping(diagnostic, joint_spec);
      Vector3d axis_J = ExtractJointAxis(diagnostic, joint_spec);
      std::optional<std::tuple<double, double, double, double>> joint_limits =
          ParseJointLimits(diagnostic, joint_spec);
      if (!joint_limits.has_value()) return false;
      std::tie(lower_limit, upper_limit, velocity_limit, acceleration_limit) =
          *joint_limits;
      const auto& joint = plant->AddJoint<PrismaticJoint>(
          joint_spec.Name(), parent_body, X_PJ, child_body, X_CJ, axis_J,
          lower_limit, upper_limit, damping);
      plant->get_mutable_joint(joint.index())
          .set_velocity_limits(Vector1d(-velocity_limit),
                               Vector1d(velocity_limit));
      plant->get_mutable_joint(joint.index())
          .set_acceleration_limits(Vector1d(-acceleration_limit),
                                   Vector1d(acceleration_limit));
      AddJointActuatorFromSpecification(diagnostic, joint_spec, joint, plant);
      AddPrismaticSpringFromSpecification(diagnostic, joint_spec, joint, plant);
      break;
    }
    case sdf::JointType::REVOLUTE: {
      const double damping = ParseJointDamping(diagnostic, joint_spec);
      Vector3d axis_J = ExtractJointAxis(diagnostic, joint_spec);
      std::optional<std::tuple<double, double, double, double>> joint_limits =
          ParseJointLimits(diagnostic, joint_spec);
      if (!joint_limits.has_value()) return false;
      std::tie(lower_limit, upper_limit, velocity_limit, acceleration_limit) =
          *joint_limits;
      const auto& joint = plant->AddJoint<RevoluteJoint>(
          joint_spec.Name(), parent_body, X_PJ, child_body, X_CJ, axis_J,
          lower_limit, upper_limit, damping);
      plant->get_mutable_joint(joint.index())
          .set_velocity_limits(Vector1d(-velocity_limit),
                               Vector1d(velocity_limit));
      plant->get_mutable_joint(joint.index())
          .set_acceleration_limits(Vector1d(-acceleration_limit),
                                   Vector1d(acceleration_limit));
      AddJointActuatorFromSpecification(diagnostic, joint_spec, joint, plant);
      if (!AddRevoluteSpringFromSpecification(diagnostic, joint_spec, joint,
                                              plant)) {
        return false;
      }
      break;
    }
    case sdf::JointType::UNIVERSAL: {
      const double damping = ParseJointDamping(diagnostic, joint_spec);
      // In Drake's implementation of universal joint, the rotation axes are
      // built into the frames M and F; the first rotation is about Fx and the
      // second is about My. Therefore, we can't arbitrarily set M and F to be
      // J. We have to be more careful on how we define F and M so that the
      // joint imposes the correct kinematics.

      // Construct frame I and find X_PI and X_CI. See definition of frame I in
      // the class doc of UniversalJoint.
      auto [Ix_J, Iy_J] = ExtractJointAxisAndAxis2(diagnostic, joint_spec);
      // Safe to normalize as libsdformat parser would have generated an error
      // if the axes are zero.
      Ix_J.normalize();
      Iy_J.normalize();
      const Vector3d Iz_J = Ix_J.cross(Iy_J);
      Matrix3d R_JI;
      R_JI.col(0) = Ix_J;
      R_JI.col(1) = Iy_J;
      R_JI.col(2) = Iz_J;
      // We require that axis and axis2 are orthogonal. As a result, R_JI should
      // be a valid rotation matrix.
      if (!math::RotationMatrixd::IsValid(R_JI)) {
        std::string message =
            fmt::format("axis and axis2 must be orthogonal for joint '{}'",
                        joint_spec.Name());
        diagnostic.Error(joint_spec.Element(), std::move(message));
      } else {
        const RigidTransformd X_JI(math::RotationMatrix<double>{R_JI});
        // The value of X_PJ is the identity if X_PJ == nullopt.
        const RigidTransformd X_PI = X_PJ.has_value() ? (*X_PJ) * X_JI : X_JI;
        const RigidTransformd X_CI = X_CJ * X_JI;
        // Frames M and F should both coincide with I when rotation angles are
        // zero.
        const RigidTransformd& X_PF = X_PI;
        const RigidTransformd& X_CM = X_CI;
        const auto& joint = plant->AddJoint<UniversalJoint>(
            joint_spec.Name(), parent_body, X_PF, child_body, X_CM, damping);
        // At most, this prints a warning (it does not add an actuator).
        AddJointActuatorFromSpecification(diagnostic, joint_spec, joint, plant);
      }
      break;
    }
    case sdf::JointType::BALL: {
      const double damping = ParseJointDamping(diagnostic, joint_spec);
      const auto& joint = plant->AddJoint<BallRpyJoint>(
          joint_spec.Name(), parent_body, X_PJ, child_body, X_CJ, damping);
      // At most, this prints a warning (it does not add an actuator).
      AddJointActuatorFromSpecification(diagnostic, joint_spec, joint, plant);
      break;
    }
    case sdf::JointType::CONTINUOUS: {
      const double damping = ParseJointDamping(diagnostic, joint_spec);
      Vector3d axis_J = ExtractJointAxis(diagnostic, joint_spec);
      std::optional<std::tuple<double, double, double, double>> joint_limits =
          ParseJointLimits(diagnostic, joint_spec);
      if (!joint_limits.has_value()) return false;
      std::tie(lower_limit, upper_limit, velocity_limit, acceleration_limit) =
          *joint_limits;
      const auto& joint =
          plant->AddJoint<RevoluteJoint>(joint_spec.Name(), parent_body, X_PJ,
                                         child_body, X_CJ, axis_J, damping);
      plant->get_mutable_joint(joint.index())
          .set_velocity_limits(Vector1d(-velocity_limit),
                               Vector1d(velocity_limit));
      plant->get_mutable_joint(joint.index())
          .set_acceleration_limits(Vector1d(-acceleration_limit),
                                   Vector1d(acceleration_limit));
      AddJointActuatorFromSpecification(diagnostic, joint_spec, joint, plant);
      if (!AddRevoluteSpringFromSpecification(diagnostic, joint_spec, joint,
                                              plant)) {
        return false;
      }
      break;
    }
    case sdf::JointType::SCREW: {
      const double damping = ParseJointDamping(diagnostic, joint_spec);
      // The ScrewThreadPitch() API uses the same representation as
      // Drake's ScrewJoint class (meters / revolution, right-handed).
      const double screw_thread_pitch = joint_spec.ScrewThreadPitch();
      Vector3d axis_J = ExtractJointAxis(diagnostic, joint_spec);
      const auto& joint = plant->AddJoint<ScrewJoint>(
          joint_spec.Name(), parent_body, X_PJ, child_body, X_CJ, axis_J,
          screw_thread_pitch, damping);
      AddJointActuatorFromSpecification(diagnostic, joint_spec, joint, plant);
      break;
    }
    case sdf::JointType::GEARBOX: {
      // TODO(jwnimmer-tri) Demote this to a warning, possibly adding a
      // RevoluteJoint as an approximation (stopgap) in that case.
      std::string message =
          fmt::format("Joint type (gearbox) not supported for joint '{}'.",
                      joint_spec.Name());
      diagnostic.Error(joint_spec.Element(), std::move(message));
      break;
    }
    case sdf::JointType::REVOLUTE2: {
      // TODO(jwnimmer-tri) Demote this to a warning, possibly adding a
      // UniversalJoint as an approximation (stopgap) in that case.
      std::string message =
          fmt::format("Joint type (revolute2) not supported for joint '{}'.",
                      joint_spec.Name());
      diagnostic.Error(joint_spec.Element(), std::move(message));
      break;
    }
    case sdf::JointType::INVALID: {
      // In probably all cases, libsdformat will have already detected
      // this error and so Drake won't get an INVALID joint.
      DRAKE_UNREACHABLE();
    }
  }
  joint_types->insert(joint_spec.Type());
  return true;
}

// Helper method to parse a custom drake:mimic tag.
bool ParseMimicTag(const SDFormatDiagnostic& diagnostic,
                   const sdf::Joint& joint_spec,
                   ModelInstanceIndex model_instance,
                   MultibodyPlant<double>* plant) {
  if (!joint_spec.Element()->HasElement("drake:mimic")) return true;

  if (!plant->is_discrete() ||
      plant->get_discrete_contact_solver() != DiscreteContactSolver::kSap) {
    diagnostic.Warning(
        joint_spec.Element(),
        fmt::format("Joint '{}' specifies a drake:mimic element that will be "
                    "ignored. Mimic elements are currently only supported by "
                    "MultibodyPlant with a discrete time step and using "
                    "DiscreteContactSolver::kSap.",
                    joint_spec.Name()));
    return true;
  }

  sdf::ElementPtr mimic_node = joint_spec.Element()->GetElement("drake:mimic");

  if (!mimic_node->HasAttribute("joint")) {
    diagnostic.Error(
        mimic_node, fmt::format("Joint '{}' drake:mimic element is missing the "
                                "required 'joint' attribute.",
                                joint_spec.Name()));
    return false;
  }

  const std::string joint_to_mimic = mimic_node->Get<std::string>("joint");

  if (!plant->HasJointNamed(joint_to_mimic, model_instance)) {
    diagnostic.Error(
        mimic_node,
        fmt::format("Joint '{}' drake:mimic element specifies joint '{}' which"
                    " does not exist.",
                    joint_spec.Name(), joint_to_mimic));
    return false;
  }

  if (joint_to_mimic == joint_spec.Name()) {
    diagnostic.Error(mimic_node,
                     fmt::format("Joint '{}' drake:mimic element specifies "
                                 "joint '{}'. Joints cannot mimic themselves.",
                                 joint_spec.Name(), joint_to_mimic));
    return false;
  }

  if (!mimic_node->HasAttribute("multiplier")) {
    diagnostic.Error(
        mimic_node, fmt::format("Joint '{}' drake:mimic element is missing the "
                                "required 'multiplier' attribute.",
                                joint_spec.Name()));
    return false;
  }

  if (!mimic_node->HasAttribute("offset")) {
    diagnostic.Error(
        mimic_node, fmt::format("Joint '{}' drake:mimic element is missing the "
                                "required 'offset' attribute.",
                                joint_spec.Name()));
    return false;
  }

  const double gear_ratio = mimic_node->Get<double>("multiplier");
  const double offset = mimic_node->Get<double>("offset");

  const Joint<double>& joint0 =
      plant->GetJointByName(joint_spec.Name(), model_instance);
  const Joint<double>& joint1 =
      plant->GetJointByName(joint_to_mimic, model_instance);

  if (joint0.num_velocities() != 1 || joint1.num_velocities() != 1) {
    // The URDF documentation is ambiguous as to whether multi-dof joints
    // are supported by the mimic tag (which the drake:mimic tag is analogous
    // to). So we only raise a warning, not an error.
    diagnostic.Warning(
        mimic_node,
        fmt::format("Drake only supports the drake:mimic element for "
                    "single-dof joints. The joint '{}' (with {} "
                    "dofs) is attempting to mimic joint '{}' (with "
                    "{} dofs). The drake:mimic element will be ignored.",
                    joint0.name(), joint0.num_velocities(), joint_to_mimic,
                    joint1.num_velocities()));
  } else {
    plant->AddCouplerConstraint(joint0, joint1, gear_ratio, offset);
  }

  return true;
}

// Helper method to load an SDF file and read the contents into an sdf::Root
// object.
[[nodiscard]] sdf::Errors LoadSdf(const SDFormatDiagnostic& diagnostic,
                                  sdf::Root* root,
                                  const DataSource& data_source,
                                  const sdf::ParserConfig& parser_config) {
  sdf::Errors errors;
  if (data_source.IsFilename()) {
    const std::string full_path = data_source.GetAbsolutePath();
    errors = root->Load(full_path, parser_config);
  } else {
    errors = root->LoadSdfString(data_source.contents(), parser_config);
  }

  if (errors.empty()) {
    const std::set<std::string> supported_root_elements{"model", "world"};

    CheckSupportedElements(diagnostic, root->Element(),
                           supported_root_elements);
  }
  return errors;
}

struct LinkInfo {
  const RigidBody<double>* body{};
  RigidTransformd X_WL;
};

// If `element`'s name has a "drake:" prefix, strip the prefix and then explore
// `element`'s children, performing the same operation. Returns `true` upon
// success.
bool DrakeVisualToSdfVisualRecurse(const SDFormatDiagnostic& diagnostic,
                                   sdf::ElementPtr element) {
  const std::string& tag_name = element->GetName();
  if (!tag_name.starts_with("drake:")) {
    diagnostic.Error(element,
                     "All tags under a drake-namespaced tag must likewise be "
                     "drake-namespaced.");
    return false;
  }
  if (tag_name == "drake:perception_properties" ||
      tag_name == "drake:illustration_properties") {
    // These drake-only tags should remain untouched to be processed later.
    // Doing nothing is success.
    return true;
  }
  element->SetName(tag_name.substr(6));
  for (sdf::ElementPtr child = element->GetFirstElement(); child != nullptr;
       child = child->GetNextElement()) {
    const bool success = DrakeVisualToSdfVisualRecurse(diagnostic, child);
    if (!success) return success;
  }
  return true;
}

// Search for all _element_ trees rooted at <drake:visual> tags and convert them
// to <visual> tags, inserting them into `link`.
void DrakeVisualToSdfVisual(const SDFormatDiagnostic& diagnostic,
                            const sdf::ParserConfig& parser_config,
                            sdf::Link* link) {
  sdf::ElementPtr link_element = link->Element();
  sdf::ElementPtr drake_visual = link_element->FindElement("drake:visual");
  while (drake_visual != nullptr) {
    if (DrakeVisualToSdfVisualRecurse(diagnostic, drake_visual)) {
      // Stash an attribute into the element that we can use later to recognize
      // the new <visual> came from <drake:visual>.
      drake_visual->AddAttribute(kIsDrakeNamespaceAttr, "bool", "true",
                                 /* required */ false);
      sdf::Visual visual;
      visual.Load(drake_visual, parser_config);
      sdf::ElementPtr next_visual =
          drake_visual->GetNextElement("drake:visual");
      link_element->RemoveChild(drake_visual);
      link->AddVisual(visual);
      drake_visual = next_visual;
    } else {
      // Generally, if we return `false`, it's because there's an error in the
      // conversion and the parsing should stop. But we'll pretend to proceed
      // for diagnostics that don't throw on error.
      drake_visual = drake_visual->GetNextElement("drake:visual");
    }
  }
}

// Immediately after parsing the .sdf file, this gives Drake a chance to
// massage the intermediate representation. This is used to account for
// Drake-specific artifacts in the file in a way that facilitates translation
// into Drake constructs.
void DrakifyModel(const SDFormatDiagnostic& diagnostic,
                  const sdf::ParserConfig& parser_config, sdf::Model* model) {
  // Translate <drake:visual> into a tweaked <visual>.
  for (uint64_t link_index = 0; link_index < model->LinkCount(); ++link_index) {
    sdf::Link* link = model->LinkByIndex(link_index);
    DrakeVisualToSdfVisual(diagnostic, parser_config, link);
  }
}

bool IsDeformableLink(const sdf::Link& link) {
  return link.Element()->HasElement("drake:deformable_properties");
}

Eigen::Vector3d ParseVector3(const SDFormatDiagnostic& diagnostic,
                             const sdf::ElementPtr node,
                             const char* element_name) {
  if (!node->HasElement(element_name)) {
    std::string message =
        fmt::format("<{}>: Unable to find the <{}> child tag.", node->GetName(),
                    element_name);
    diagnostic.Error(node, message);
    return Eigen::Vector3d::Zero();
  }

  auto value = node->Get<gz::math::Vector3d>(element_name);

  return ToVector3(value);
}

// Structure to hold wall boundary condition data during parsing
struct WallBoundaryCondition {
  Eigen::Vector3d p_WQ;  // Position of point Q on the plane in world frame
  Eigen::Vector3d n_W;   // Outward normal to the half space in world frame
};

// Helper function to parse wall boundary conditions from an element
void ParseWallBoundaryConditions(
    const sdf::ElementPtr element,
    std::vector<WallBoundaryCondition>* boundary_conditions,
    const SDFormatDiagnostic& diagnostic) {
  for (sdf::ElementPtr wall_boundary_cond_element =
           element->GetElement("drake:wall_boundary_condition");
       wall_boundary_cond_element != nullptr;
       wall_boundary_cond_element = wall_boundary_cond_element->GetNextElement(
           "drake:wall_boundary_condition")) {
    // Parse point_on_plane and outward_normal child tags.
    const Eigen::Vector3d p_WQ = ParseVector3(
        diagnostic, wall_boundary_cond_element, "drake:point_on_plane");
    const Eigen::Vector3d n_W_raw = ParseVector3(
        diagnostic, wall_boundary_cond_element, "drake:outward_normal");

    // Validate normal vector is not zero.
    if (!(n_W_raw.norm() > 1e-10)) {
      diagnostic.Error(
          wall_boundary_cond_element,
          "Outward normal vector cannot be zero in <drake:outward_normal>");
      continue;
    }

    WallBoundaryCondition boundary_cond;
    boundary_cond.p_WQ = p_WQ;
    boundary_cond.n_W = n_W_raw.normalized();
    boundary_conditions->push_back(boundary_cond);
  }
}

// Helper that loads `<drake:deformable_properties>` into a config.
// @param[in] link         The SDF link to load the property for.
// @param[in, out] config  On input, it's a default config. On output, it's the
//                         config with the properties loaded from the given
//                         link.
// @pre link is a deformable link.
void LoadDeformableConfig(const sdf::Link& link,
                          fem::DeformableBodyConfig<double>* config,
                          const SDFormatDiagnostic& diagnostic) {
  DRAKE_DEMAND(IsDeformableLink(link));
  const sdf::ElementPtr property_element =
      link.Element()->GetElement("drake:deformable_properties");
  // clang-format off
    const std::set<std::string> supported_proximity_elements{
        "drake:youngs_modulus",
        "drake:poissons_ratio",
        "drake:mass_damping",
        "drake:stiffness_damping",
        "drake:mass_density",
        "drake:material_model"};
  // clang-format on
  CheckSupportedElements(diagnostic, property_element,
                         supported_proximity_elements);

  if (property_element->HasElement("drake:youngs_modulus")) {
    const double val = property_element->Get<double>("drake:youngs_modulus");
    if (!(val > 0)) {
      diagnostic.Error(property_element, "Young's modulus must be positive.");
      return;
    }
    config->set_youngs_modulus(val);
  }
  if (property_element->HasElement("drake:poissons_ratio")) {
    const double val = property_element->Get<double>("drake:poissons_ratio");
    if (!(val > -1 && val < 0.5)) {
      diagnostic.Error(property_element,
                       "Poisson's ratio must be in the range (-1, 0.5).");
      return;
    }
    config->set_poissons_ratio(val);
  }
  if (property_element->HasElement("drake:mass_damping")) {
    const double val = property_element->Get<double>("drake:mass_damping");
    if (!(val >= 0)) {
      diagnostic.Error(property_element, "Mass damping must be non-negative.");
      return;
    }
    config->set_mass_damping_coefficient(val);
  }
  if (property_element->HasElement("drake:stiffness_damping")) {
    const double val = property_element->Get<double>("drake:stiffness_damping");
    if (!(val >= 0)) {
      diagnostic.Error(property_element,
                       "Stiffness damping must be non-negative.");
      return;
    }
    config->set_stiffness_damping_coefficient(val);
  }
  if (property_element->HasElement("drake:mass_density")) {
    const double val = property_element->Get<double>("drake:mass_density");
    if (!(val > 0)) {
      diagnostic.Error(property_element, "Mass density must be positive.");
      return;
    }
    config->set_mass_density(val);
  }
  if (property_element->HasElement("drake:material_model")) {
    const std::string mm =
        property_element->Get<std::string>("drake:material_model");
    if (mm == "linear_corotated") {
      config->set_material_model(fem::MaterialModel::kLinearCorotated);
    } else if (mm == "corotated") {
      config->set_material_model(fem::MaterialModel::kCorotated);
    } else if (mm == "neohookean") {
      config->set_material_model(fem::MaterialModel::kNeoHookean);
    } else if (mm == "linear") {
      config->set_material_model(fem::MaterialModel::kLinear);
    } else {
      diagnostic.Error(
          property_element,
          fmt::format(
              "Invalid <drake:material_model> value {}. Must be 'linear', "
              "'linear_corotated', `neohookean`, or 'corotated'.",
              mm));
    }
  }
}

// Parses one <link> into a deformable body. Returns true if the parser should
// keep parsing the rest of the links, false if an error is encountered and the
// parser should abort.
bool AddDeformableLinkFromSpecification(const SDFormatDiagnostic& diag,
                                        const ModelInstanceIndex model_instance,
                                        const sdf::Link& link,
                                        const math::RigidTransformd& X_WM,
                                        MultibodyPlant<double>* plant,
                                        const PackageMap& package_map,
                                        const std::string& root_dir) {
  const sdf::ElementPtr link_element = link.Element();
  DRAKE_DEMAND(!plant->is_finalized());
  if (!plant->geometry_source_is_registered()) {
    diag.Warning(link_element,
                 "Cannot add a deformable link to a plant without a registered "
                 "geometry source. The deformable link is ignored.");
    return true;
  }

  // Supported child tags inside <link>.
  CheckSupportedElements(
      diag, link_element,
      {"pose", "collision", "visual", "drake:deformable_properties",
       "drake:wall_boundary_condition"});

  // Config
  fem::DeformableBodyConfig<double> config;
  LoadDeformableConfig(link, &config, diag);

  // Wall boundary conditions
  std::vector<WallBoundaryCondition> boundary_conditions;
  ParseWallBoundaryConditions(link_element, &boundary_conditions, diag);

  ResolveFilename resolve_filename =
      [&package_map, &root_dir, &link](
          const SDFormatDiagnostic& inner_diagnostic, std::string uri) {
        const ResolveUriResult resolved =
            ResolveUri(inner_diagnostic.MakePolicyForNode(*link.Element()), uri,
                       package_map, root_dir);
        return resolved.GetStringPathIfExists();
      };

  // Collision (exactly one).
  if (link.CollisionCount() != 1) {
    diag.Error(
        link_element,
        "Each deformable <link> must have exactly one <collision> element.");
    return false;
  }

  const sdf::Collision& sdf_collision = *link.CollisionByIndex(0);
  const sdf::Geometry& collision_geometry = *sdf_collision.Geom();
  sdf::ElementPtr collision_geometry_element = collision_geometry.Element();

  const std::set<std::string> supported_collision_geometry_elements{"mesh"};
  CheckSupportedElements(diag, collision_geometry_element,
                         supported_collision_geometry_elements);

  auto shape =
      MakeShapeFromSdfGeometry(diag, collision_geometry, resolve_filename);
  // Error already reported in MakeShapeFromSdfGeometry.
  if (!shape) return false;

  // Pose
  const RigidTransformd X_ML = ResolveRigidTransform(diag, link.SemanticPose());
  const RigidTransformd X_WL = X_WM * X_ML;

  // Now create the geometry instance.
  auto geometry_instance =
      std::make_unique<GeometryInstance>(X_WL, std::move(shape), link.Name());

  // Parse proximity properties from <drake:proximity_properties> if they are
  // legally specified. Otherwise, add a default proximity property.
  if (auto proximity_props =
          MakeProximityForDeformableCollision(diag, sdf_collision)) {
    geometry_instance->set_proximity_properties(*proximity_props);
  } else {
    geometry::ProximityProperties default_proximity_props;
    default_proximity_props.AddProperty("material", "coulomb_friction",
                                        default_friction());
    geometry_instance->set_proximity_properties(default_proximity_props);
  }

  // Optional embedded‑mesh perception – allow **at most one** <visual>.
  // TODO(xuchenhan-tri): Support multiple <visual> elements.
  if (link.VisualCount() > 1) {
    diag.Error(link_element,
               "A deformable <link> may have at most one <visual> element.");
    return false;
  }

  if (link.VisualCount() == 1) {
    const sdf::Visual& sdf_visual = *link.VisualByIndex(0);
    // Supported child tags inside <visual>.
    CheckSupportedElements(
        diag, sdf_visual.Element(),
        {"geometry", "material", "drake:perception_properties",
         "drake:illustration_properties", "drake:accepting_renderer"});
    VisualProperties visual_props =
        MakeVisualPropertiesFromSdfVisual(diag, sdf_visual, resolve_filename);
    if (visual_props.illustration.has_value()) {
      geometry_instance->set_illustration_properties(
          *visual_props.illustration);
    }
    std::optional<geometry::PerceptionProperties> perception_props =
        visual_props.perception;
    // Override the visual geometry if a mesh is specified. Otherwise, fall back
    // to the default visual geometry (the surface of the simulation mesh).
    if (sdf_visual.Element()->HasElement("geometry")) {
      const sdf::Geometry& visual_geometry = *sdf_visual.Geom();
      sdf::ElementPtr visual_geometry_element = visual_geometry.Element();
      const std::set<std::string> supported_visual_geometry_elements{"empty",
                                                                     "mesh"};
      CheckSupportedElements(diag, visual_geometry_element,
                             supported_visual_geometry_elements);
      if (visual_geometry_element->HasElement("mesh")) {
        if (!perception_props.has_value()) {
          diag.Warning(
              sdf_visual.Element(),
              "The <visual> element in a deformable <link> specified a "
              "non-empty <geometry> element, but it's ignored because "
              "non-empty visual geometries for deformables are only supported "
              "as rendering meshes; however, no `drake:perception_properties` "
              "has been specified for this visual geometry.");
        } else {
          const std::string uri =
              visual_geometry_element->GetElement("mesh")->Get<std::string>(
                  "uri");
          const std::string file_name = resolve_filename(diag, uri);
          perception_props->AddProperty("deformable", "embedded_mesh",
                                        file_name);
        }
      }
    }
    if (perception_props) {
      geometry_instance->set_perception_properties(*perception_props);
    }
  }

  DeformableModel<double>& deformable_model = plant->mutable_deformable_model();
  // Right now resolution hint is not used because we only parse meshes. When we
  // support primitive geometries, the resolution hint needs to be meaningfully
  // parsed.
  const double dummy_resolution_hint = 1.0;
  const DeformableBodyId body_id = deformable_model.RegisterDeformableBody(
      std::move(geometry_instance), model_instance, config,
      dummy_resolution_hint);

  // Apply wall boundary conditions after body registration
  for (const WallBoundaryCondition& boundary_cond : boundary_conditions) {
    deformable_model.SetWallBoundaryCondition(body_id, boundary_cond.p_WQ,
                                              boundary_cond.n_W);
  }

  return true;
}

// Helper method to add a rigid body (along with its geometries) to a
// MultibodyPlant given an sdf::Link specification object.
std::optional<LinkInfo> AddRigidLinkFromSpecification(
    const SDFormatDiagnostic& diagnostic,
    const ModelInstanceIndex model_instance, const sdf::Link& link,
    const RigidTransformd& X_WM, MultibodyPlant<double>* plant,
    const PackageMap& package_map, const std::string& root_dir) {
  std::optional<LinkInfo> link_info;

  const std::set<std::string> supported_link_elements{
      "drake:visual", "collision", "gravity", "inertial",
      "kinematic",    "pose",      "visual"};

  sdf::ElementPtr link_element = link.Element();

  CheckSupportedElements(diagnostic, link_element, supported_link_elements);

  CheckSupportedElementValue(diagnostic, link_element, "kinematic", "false");
  CheckSupportedElementValue(diagnostic, link_element, "gravity", "true");

  // Get the link's inertia relative to the Bcm frame.
  // sdf::Link::Inertial() provides a representation for the SpatialInertia
  // M_Bcm_Bi of body B, about its center of mass Bcm, and expressed in an
  // inertial frame Bi as defined in <inertial> <pose></pose> </inertial>.
  // Per SDF specification, Bi's origin is at the COM Bcm, but Bi is not
  // necessarily aligned with B.
  const gz::math::Inertiald& Inertial_Bcm_Bi = link.Inertial();

  const SpatialInertia<double> M_BBo_B =
      ExtractSpatialInertiaAboutBoExpressedInB(diagnostic, link_element,
                                               Inertial_Bcm_Bi);

  // Add a rigid body to model each link.
  const RigidBody<double>& body =
      plant->AddRigidBody(link.Name(), model_instance, M_BBo_B);

  // Register information.
  const RigidTransformd X_ML =
      ResolveRigidTransform(diagnostic, link.SemanticPose());
  const RigidTransformd X_WL = X_WM * X_ML;
  link_info = LinkInfo{&body, X_WL};

  // Provisionally record the preferred World pose of this link. This will
  // be ignored unless this link turns out to be a floating base body,
  // meaning that no user-defined joint connects it to a parent link.
  plant->SetDefaultFloatingBaseBodyPose(body, X_WL);

  const std::set<std::string> supported_geometry_elements{
      "box",       "capsule", "cylinder", "drake:capsule", "drake:ellipsoid",
      "ellipsoid", "empty",   "mesh",     "plane",         "sphere"};

  if (plant->geometry_source_is_registered()) {
    ResolveFilename resolve_filename =
        [&package_map, &root_dir, &link_element](
            const SDFormatDiagnostic& inner_diagnostic, std::string uri) {
          const ResolveUriResult resolved =
              ResolveUri(inner_diagnostic.MakePolicyForNode(*link_element), uri,
                         package_map, root_dir);
          return resolved.GetStringPathIfExists();
        };

    for (uint64_t visual_index = 0; visual_index < link.VisualCount();
         ++visual_index) {
      const sdf::Visual& sdf_visual = *link.VisualByIndex(visual_index);
      const sdf::Geometry& sdf_geometry = *sdf_visual.Geom();

      sdf::ElementPtr geometry_element = sdf_geometry.Element();
      CheckSupportedElements(diagnostic, geometry_element,
                             supported_geometry_elements);

      const RigidTransformd X_LG =
          ResolveRigidTransform(diagnostic, sdf_visual.SemanticPose());
      unique_ptr<GeometryInstance> geometry_instance =
          MakeGeometryInstanceFromSdfVisual(diagnostic, sdf_visual,
                                            resolve_filename, X_LG);
      // No instance may simply mean there was a visual we should skip and
      // we move on to the next. If there is a _real_ problem, we assume an
      // error was reported to diagnostic (and it responds appropriately).
      if (geometry_instance == nullptr) continue;

      // If we have a geometry instance, it has at least one set of visual
      // properties.
      DRAKE_DEMAND(geometry_instance->illustration_properties() != nullptr ||
                   geometry_instance->perception_properties() != nullptr);

      plant->RegisterVisualGeometry(body, std::move(geometry_instance));
    }

    for (uint64_t collision_index = 0; collision_index < link.CollisionCount();
         ++collision_index) {
      const sdf::Collision& sdf_collision =
          *link.CollisionByIndex(collision_index);
      const sdf::Geometry& sdf_geometry = *sdf_collision.Geom();

      sdf::ElementPtr geometry_element = sdf_geometry.Element();
      CheckSupportedElements(diagnostic, geometry_element,
                             supported_geometry_elements);

      std::optional<std::unique_ptr<geometry::Shape>> shape =
          MakeShapeFromSdfGeometry(diagnostic, sdf_geometry, resolve_filename);
      if (!shape.has_value()) return std::nullopt;
      if (*shape != nullptr) {
        const RigidTransformd X_LG =
            ResolveRigidTransform(diagnostic, sdf_collision.SemanticPose());
        const RigidTransformd X_LC =
            MakeGeometryPoseFromSdfCollision(sdf_collision, X_LG);
        std::optional<geometry::ProximityProperties> props =
            MakeProximityPropertiesForCollision(diagnostic, sdf_collision);
        if (!props.has_value()) return std::nullopt;
        plant->RegisterCollisionGeometry(
            body, X_LC, **shape, sdf_collision.Name(), std::move(*props));
      }
    }
  }
  return link_info;
}

const Frame<double>& AddFrameFromSpecification(
    const SDFormatDiagnostic& diagnostic, const sdf::Frame& frame_spec,
    ModelInstanceIndex model_instance, const Frame<double>& default_frame,
    MultibodyPlant<double>* plant) {
  const Frame<double>* parent_frame{};
  const RigidTransformd X_PF = ResolveRigidTransform(
      diagnostic, frame_spec.SemanticPose(), frame_spec.AttachedTo());
  if (frame_spec.AttachedTo().empty()) {
    parent_frame = &default_frame;
  } else {
    const std::string attached_to_absolute_name =
        ScopedName::Join(
            GetInstanceScopeNameIgnoringWorld(*plant, model_instance),
            frame_spec.AttachedTo())
            .to_string();

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
      if (plant->HasFrameNamed(local_name, parent_model_instance)) {
        parent_frame =
            &plant->GetFrameByName(local_name, parent_model_instance);
      } else {
        // If there is no frame named `local_name`, the `attached_to` attribute
        // must be pointing to something we don't create implicit frames for in
        // Drake. Currently these are models and joints. Models are handled in
        // the first `if` block, so we're dealing with joints here. Since joints
        // may end up in a model instance different from the model in which they
        // were defined, we don't bother to find the joint and use its child
        // frame. Instead we ask libsdformat to resolve the body associated with
        // whatever is referenced by the `attached_to` attribute. Since this is
        // a body, we're assured that its implicit frame exists in the plant.
        std::string resolved_attached_to_body_name;
        sdf::Errors errors =
            frame_spec.ResolveAttachedToBody(resolved_attached_to_body_name);
        diagnostic.PropagateErrors(errors);
        const std::string resolved_attached_to_body_absolute_name =
            ScopedName::Join(
                GetInstanceScopeNameIgnoringWorld(*plant, model_instance),
                resolved_attached_to_body_name)
                .to_string();
        parent_frame = parsing::GetScopedFrameByNameMaybe(
            *plant, resolved_attached_to_body_absolute_name);
      }
    }
  }
  // Libsdformat will have already detected this error and
  // at this point the parent_frame wouldn't be a nullptr
  DRAKE_DEMAND(parent_frame != nullptr);

  const Frame<double>& frame =
      plant->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
          frame_spec.Name(), *parent_frame, X_PF));
  return frame;
}

bool ParseBoolean(const SDFormatDiagnostic& diagnostic,
                  const sdf::ElementPtr node, const char* element_name) {
  // In the only existing call site, presence has already been checked.
  DRAKE_DEMAND(node->HasElement(element_name));

  const std::string value = node->Get<std::string>(element_name);

  if (value != "true" && value != "false") {
    std::string message =
        fmt::format("<{}>: boolean node contains non-boolean value '{}'.",
                    element_name, value);
    diagnostic.Error(node, message);
    return false;
  }

  return value == "true" ? true : false;
}

std::optional<double> ParseDouble(
    const SDFormatDiagnostic& diagnostic, const sdf::ElementPtr node,
    const char* element_name,
    const std::function<bool(double)> validator = nullptr) {
  if (!node->HasElement(element_name)) {
    std::string message =
        fmt::format("<{}>: Unable to find the <{}> child tag.", node->GetName(),
                    element_name);
    diagnostic.Error(node, message);
    return {};
  }

  const double value = node->Get<double>(element_name);
  if (validator == nullptr || validator(value)) {
    return value;
  }

  return {};
}

bool ParseDrakeCurves(const SDFormatDiagnostic& diagnostic,
                      const sdf::ElementPtr node, std::vector<double>* breaks,
                      std::vector<double>* turning_rates) {
  if (!node->HasElement("drake:curves")) {
    std::string message = fmt::format(
        "<{}>: Unable to find the <drake:curves> child node.", node->GetName());
    diagnostic.Error(node, message);
    return false;
  }

  breaks->clear();
  turning_rates->clear();
  breaks->push_back(0.0);

  for (sdf::ElementPtr curve_node =
           node->GetElement("drake:curves")->GetFirstElement();
       curve_node != NULL; curve_node = curve_node->GetNextElement()) {
    const auto name = curve_node->GetName();
    double length{};
    double angle{};
    if (name == "drake:line_segment") {
      std::optional<double> maybe_length =
          ParseDouble(diagnostic, curve_node, "drake:length");
      if (!maybe_length.has_value()) {
        return false;
      }
      length = *maybe_length;
      if (length <= 0.0) {
        std::string message = fmt::format(
            "<{}>: A drake:line_segment node has a 0 or "
            "negative drake:length.",
            curve_node->GetName());
        diagnostic.Error(node, message);
        return false;
      }
    } else if (name == "drake:circular_arc") {
      std::optional<double> maybe_radius =
          ParseDouble(diagnostic, curve_node, "drake:radius");
      if (!maybe_radius.has_value()) {
        return false;
      }
      double radius = *maybe_radius;
      if (radius <= 0.0) {
        std::string message = fmt::format(
            "<{}>: A drake:circular_arc node has a 0 or "
            "negative drake:radius.",
            curve_node->GetName());
        diagnostic.Error(node, message);
        return false;
      }
      std::optional<double> maybe_angle =
          ParseDouble(diagnostic, curve_node, "drake:angle");
      if (!maybe_angle.has_value()) {
        return false;
      }
      angle = *maybe_angle;
      length = std::abs(angle) * radius;
    } else {
      std::string message = fmt::format(
          "<{}>: drake:curves node contains an invalid child node <{}>.",
          node->GetName(), name);
      diagnostic.Error(node, message);
      return false;
    }
    breaks->push_back(breaks->back() + length);
    turning_rates->push_back(angle / length);
  }
  return true;
}

const Frame<double>* ParseFrame(const SDFormatDiagnostic& diagnostic,
                                const sdf::ElementPtr node,
                                ModelInstanceIndex model_instance,
                                MultibodyPlant<double>* plant,
                                const char* element_name) {
  if (!node->HasElement(element_name)) {
    std::string message =
        fmt::format("<{}>: Unable to find the <{}> child tag.", node->GetName(),
                    element_name);
    diagnostic.Error(node, std::move(message));
    return nullptr;
  }

  // TODO(rpoyner-tri): In theory, we could use
  // GetResolvedModelInstanceAndLocalName here. In practice, it doesn't turn
  // out well, because that function just allows MultibodyPlant code to throw
  // exceptions, instead of checking for bad inputs and issuing model source
  // code locatable diagnostics. Consider harmonizing the various
  // implementations with better checking and diagnostic messages.

  // Support references to frames
  // in nested models. Note: only downward nested references are permitted.
  // See http://sdformat.org/tutorials?tut=composition#composition
  const std::string current_model_name =
      plant->GetModelInstanceName(model_instance);
  const std::string frame_name = node->Get<std::string>(element_name);
  auto absolute_scoped_name = ScopedName::Join(current_model_name, frame_name);

  const std::string search_model_name(absolute_scoped_name.get_namespace());
  if (!plant->HasModelInstanceNamed(search_model_name)) {
    std::string message = fmt::format(
        "<{}>: Model instance name '{}' (implied by frame name '{}' in <{}>"
        " within model instance '{}') does not exist in the model.",
        node->GetName(), search_model_name, frame_name, element_name,
        current_model_name);
    diagnostic.Error(node, std::move(message));
    return nullptr;
  }

  ModelInstanceIndex search_model_instance =
      plant->GetModelInstanceByName(search_model_name);
  const std::string search_frame_name(absolute_scoped_name.get_element());

  if (!plant->HasFrameNamed(search_frame_name, search_model_instance)) {
    std::string message = fmt::format(
        "<{}>: Frame '{}' specified for <{}> does not exist in the model.",
        node->GetName(), frame_name, element_name);
    diagnostic.Error(node, std::move(message));
    return nullptr;
  }

  return &plant->GetFrameByName(search_frame_name, search_model_instance);
}

const RigidBody<double>* ParseBody(const SDFormatDiagnostic& diagnostic,
                                   const sdf::ElementPtr node,
                                   ModelInstanceIndex model_instance,
                                   MultibodyPlant<double>* plant,
                                   const char* element_name) {
  if (!node->HasElement(element_name)) {
    std::string message =
        fmt::format("<{}>: Unable to find the <{}> child tag.", node->GetName(),
                    element_name);
    diagnostic.Error(node, std::move(message));
    return nullptr;
  }

  const std::string body_name = node->Get<std::string>(element_name);

  if (!plant->HasBodyNamed(body_name, model_instance)) {
    std::string message = fmt::format(
        "<{}>: Body '{}' specified for <{}> does not exist in the model.",
        node->GetName(), body_name, element_name);
    diagnostic.Error(node, std::move(message));
    return nullptr;
  }

  return &plant->GetBodyByName(body_name, model_instance);
}

// TODO(eric.cousineau): Update parsing pending resolution of
// https://github.com/osrf/sdformat/issues/288
// When diagnostic policy is not set to throw it returns false on errors.
bool AddDrakeJointFromSpecification(const SDFormatDiagnostic& diagnostic,
                                    const sdf::ElementPtr node,
                                    ModelInstanceIndex model_instance,
                                    MultibodyPlant<double>* plant) {
  // clang-format off
  const std::set<std::string> supported_joint_elements{
      "drake:parent",
      "drake:child",
      "drake:damping",
      "drake:initial_tangent",
      "drake:plane_normal",
      "drake:is_periodic",
      "drake:curves",
      "drake:line_segment",
      "drake:circular_arc",
      "drake:length",
      "drake:radius",
      "drake:angle",
      "pose"};
  // clang-format on
  CheckSupportedElements(diagnostic, node, supported_joint_elements);

  if (!node->HasAttribute("type")) {
    std::string message = "<drake:joint>: Unable to find the 'type' attribute.";
    diagnostic.Error(node, std::move(message));
    return false;
  }
  const std::string joint_type = node->Get<std::string>("type");
  if (!node->HasAttribute("name")) {
    std::string message = "<drake:joint>: Unable to find the 'name' attribute.";
    diagnostic.Error(node, std::move(message));
    return false;
  }
  const std::string joint_name = node->Get<std::string>("name");

  // TODO(eric.cousineau): Add support for parsing joint pose.
  if (node->HasElement("pose")) {
    std::string message =
        "<drake:joint> does not yet support the <pose> child tag.";
    diagnostic.Error(node, std::move(message));
    return false;
  }

  const Frame<double>* parent_frame =
      ParseFrame(diagnostic, node, model_instance, plant, "drake:parent");
  if (parent_frame == nullptr) {
    return false;
  }
  const Frame<double>* child_frame =
      ParseFrame(diagnostic, node, model_instance, plant, "drake:child");
  if (child_frame == nullptr) {
    return false;
  }

  if (joint_type == "planar") {
    // TODO(eric.cousineau): Error out when there are unused tags.
    Vector3d damping = ParseVector3(diagnostic, node, "drake:damping");
    plant->AddJoint(std::make_unique<PlanarJoint<double>>(
        joint_name, *parent_frame, *child_frame, damping));
  } else if (joint_type == "curvilinear") {
    // Defaults for optional tags
    Vector3d plane_normal(0, 0, 1);
    Vector3d initial_tangent(1, 0, 0);
    bool is_periodic = false;
    double damping = 0.0;
    if (node->HasElement("drake:plane_normal")) {
      plane_normal = ParseVector3(
          diagnostic, node->GetElement("drake:plane_normal"), "xyz");
    }
    if (node->HasElement("drake:initial_tangent")) {
      initial_tangent = ParseVector3(
          diagnostic, node->GetElement("drake:initial_tangent"), "xyz");
    }
    if (node->HasElement("drake:is_periodic")) {
      is_periodic = ParseBoolean(diagnostic, node, "drake:is_periodic");
    }
    if (node->HasElement("drake:damping")) {
      std::optional<double> maybe_damping =
          ParseDouble(diagnostic, node, "drake:damping");
      if (!maybe_damping.has_value()) {
        return false;
      }
      damping = *maybe_damping;
      if (damping < 0) {
        std::string message = "ERROR: <drake:joint> '" + joint_name +
                              "' has negative value for 'damping' attribute: " +
                              std::to_string(damping);
        diagnostic.Error(node, std::move(message));
        return false;
      }
    }

    std::vector<double> breaks;
    std::vector<double> turning_rates;
    if (!ParseDrakeCurves(diagnostic, node, &breaks, &turning_rates)) {
      return false;
    }
    trajectories::PiecewiseConstantCurvatureTrajectory<double> trajectory(
        breaks, turning_rates, initial_tangent, plane_normal, Vector3d::Zero(),
        is_periodic);
    plant->AddJoint(std::make_unique<CurvilinearJoint<double>>(
        joint_name, *parent_frame, *child_frame, trajectory, damping));
  } else {
    std::string message =
        "ERROR: <drake:joint> '" + joint_name +
        "' has unrecognized value for 'type' attribute: " + joint_type;
    diagnostic.Error(node, std::move(message));
    return false;
  }
  return true;
}

const LinearBushingRollPitchYaw<double>* AddBushingFromSpecification(
    const SDFormatDiagnostic& diagnostic, const sdf::ElementPtr node,
    ModelInstanceIndex model_instance, MultibodyPlant<double>* plant) {
  const std::set<std::string> supported_bushing_elements{
      "drake:bushing_frameA",         "drake:bushing_frameC",
      "drake:bushing_force_damping",  "drake:bushing_force_stiffness",
      "drake:bushing_torque_damping", "drake:bushing_torque_stiffness"};
  CheckSupportedElements(diagnostic, node, supported_bushing_elements);

  // Functor to read a vector valued child tag with tag name: `element_name`
  // e.g. <element_name>0 0 0</element_name>
  // Reports an error if the tag does not exist.
  auto read_vector = [&diagnostic,
                      node](const char* element_name) -> Eigen::Vector3d {
    return ParseVector3(diagnostic, node, element_name);
  };

  // Functor to read a child tag with tag name: `element_name` that specifies a
  // frame name, e.g. <element_name>frame_name</element_name>
  // Reports an error if the tag does not exist or if the frame does not exist
  // in the plant.
  auto read_frame = [&diagnostic, node, model_instance,
                     plant](const char* element_name) -> const Frame<double>* {
    return ParseFrame(diagnostic, node, model_instance, plant, element_name);
  };

  return ParseLinearBushingRollPitchYaw(read_vector, read_frame, plant);
}

std::optional<MultibodyConstraintId> AddBallConstraintFromSpecification(
    const SDFormatDiagnostic& diagnostic, const sdf::ElementPtr node,
    ModelInstanceIndex model_instance, MultibodyPlant<double>* plant) {
  const std::set<std::string> supported_ball_constraint_elements{
      "drake:ball_constraint_body_A",
      "drake:ball_constraint_p_AP",
      "drake:ball_constraint_body_B",
      "drake:ball_constraint_p_BQ",
  };
  CheckSupportedElements(diagnostic, node, supported_ball_constraint_elements);

  // Functor to read a vector valued child tag with tag name: `element_name`
  // e.g. <element_name>0 0 0</element_name>
  // Reports an error if the tag does not exist.
  auto read_vector = [&diagnostic,
                      node](const char* element_name) -> Eigen::Vector3d {
    return ParseVector3(diagnostic, node, element_name);
  };

  // Functor to read a child tag with tag name: `element_name` that specifies a
  // body name, e.g. <element_name>body_name</element_name>
  // Reports an error if the tag does not exist or if the body does not exist in
  // the plant.
  auto read_body = [&diagnostic, node, model_instance, plant](
                       const char* element_name) -> const RigidBody<double>* {
    return ParseBody(diagnostic, node, model_instance, plant, element_name);
  };

  return ParseBallConstraint(read_vector, read_body, plant);
}

const LinearSpringDamper<double>* AddLinearSpringDamperFromSpecification(
    const SDFormatDiagnostic& diagnostic, const sdf::ElementPtr node,
    ModelInstanceIndex model_instance, MultibodyPlant<double>* plant) {
  const std::set<std::string> supported_elements{
      "drake:linear_spring_damper_body_A",
      "drake:linear_spring_damper_p_AP",
      "drake:linear_spring_damper_body_B",
      "drake:linear_spring_damper_p_BQ",
      "drake:linear_spring_damper_free_length",
      "drake:linear_spring_damper_stiffness",
      "drake:linear_spring_damper_damping"};
  CheckSupportedElements(diagnostic, node, supported_elements);

  // Functor to read a vector valued child tag with tag name: `element_name`
  // e.g. <element_name>0 0 0</element_name>
  // Reports an error if the tag does not exist.
  auto read_vector = [&diagnostic,
                      node](const char* element_name) -> Eigen::Vector3d {
    return ParseVector3(diagnostic, node, element_name);
  };

  // Functor to read a child tag with tag name: `element_name` that specifies a
  // body name, e.g. <element_name>body_name</element_name>
  // Reports an error if the tag does not exist or if the body does not exist in
  // the plant.
  auto read_body = [&diagnostic, node, model_instance, plant](
                       const char* element_name) -> const RigidBody<double>* {
    return ParseBody(diagnostic, node, model_instance, plant, element_name);
  };

  // Functor to read a double valued child tag with tag name: `element_name`
  // e.g. <element_name>0</element_name>
  // Returns std::nullopt if the tag does not exist.
  auto read_double = [&diagnostic,
                      node](const char* element_name) -> std::optional<double> {
    auto validator = [&diagnostic, node, element_name](double result) {
      // For drake:linear_spring_damper_free_length: require strictly positive
      if (result <= 0 && std::string(element_name) ==
                             "drake:linear_spring_damper_free_length") {
        std::string message =
            fmt::format("<{}>: The <{}> child tag must be strictly positive.",
                        node->GetName(), element_name);
        diagnostic.Error(node, std::move(message));
        return false;
      }
      // For other elements: require non-negative
      if (result < 0) {
        std::string message =
            fmt::format("<{}>: The <{}> child tag must be non-negative.",
                        node->GetName(), element_name);
        diagnostic.Error(node, std::move(message));
        return false;
      }
      return true;
    };

    return ParseDouble(diagnostic, node, element_name, validator);
  };

  return ParseLinearSpringDamper(read_vector, read_body, read_double, plant);
}

std::optional<MultibodyConstraintId> AddTendonConstraintFromSpecification(
    const SDFormatDiagnostic& diagnostic, const sdf::ElementPtr node,
    ModelInstanceIndex model_instance, MultibodyPlant<double>* plant) {
  const std::set<std::string> supported_tendon_constraint_elements{
      "drake:tendon_constraint_joint",
      "drake:tendon_constraint_offset",
      "drake:tendon_constraint_lower_limit",
      "drake:tendon_constraint_upper_limit",
      "drake:tendon_constraint_stiffness",
      "drake:tendon_constraint_damping",
  };
  CheckSupportedElements(diagnostic, node,
                         supported_tendon_constraint_elements);

  // Functor to read a scalar valued child tag with tag name: `element_name`
  // e.g. <element_name>0</element_name>
  // Reports an error if the tag does not exist.
  auto read_double = [&diagnostic,
                      node](const char* element_name) -> std::optional<double> {
    return ParseDouble(diagnostic, node, element_name);
  };

  auto next_child_element = [](const ElementNode& data_element,
                               const char* element_name) {
    return std::get<sdf::ElementPtr>(data_element)
        ->GetElementImpl(std::string(element_name));
  };
  auto next_sibling_element = [](const ElementNode& data_element,
                                 const char* element_name) {
    return std::get<sdf::ElementPtr>(data_element)
        ->GetNextElement(std::string(element_name));
  };
  // Functor to read a string valued attribute with attribute name:
  // `attribute_name` e.g. <element attribute_name="string"/>
  // Reports an error if the attribute does not exist.
  auto get_string_attribute = [&diagnostic](
                                  const ElementNode& data_element,
                                  const char* attribute_name) -> std::string {
    auto element = std::get<sdf::ElementPtr>(data_element);
    if (!element->HasAttribute(attribute_name)) {
      std::string message =
          fmt::format("The tag <{}> is missing the required attribute \"{}\"",
                      element->GetName(), attribute_name);
      diagnostic.Error(element, std::move(message));
      return {};
    }
    return std::get<sdf::ElementPtr>(data_element)
        ->Get<std::string>(attribute_name);
  };
  // Functor to read a double valued attribute with attribute name:
  // `attribute_name` e.g. <element attribute_name="0.0"/>
  // Reports an error if the attribute does not exist or is not a valid number.
  auto get_double_attribute = [&diagnostic](
                                  const ElementNode& data_element,
                                  const char* attribute_name) -> double {
    auto element = std::get<sdf::ElementPtr>(data_element);
    if (!element->HasAttribute(attribute_name)) {
      std::string message =
          fmt::format("The tag <{}> is missing the required attribute \"{}\"",
                      element->GetName(), attribute_name);
      diagnostic.Error(element, std::move(message));
      return {};
    }
    return std::get<sdf::ElementPtr>(data_element)->Get<double>(attribute_name);
  };

  return ParseTendonConstraint(
      diagnostic.MakePolicyForNode(*node), model_instance, node, read_double,
      next_child_element, next_sibling_element, get_string_attribute,
      get_double_attribute, plant);
}

// Helper to determine if two links are welded together.
bool AreWelded(const MultibodyPlant<double>& plant, const RigidBody<double>& a,
               const RigidBody<double>& b) {
  for (auto* body : plant.GetBodiesWeldedTo(a)) {
    if (body == &b) {
      return true;
    }
  }
  return false;
}

void ParseCollisionFilterGroup(const SDFormatDiagnostic& diagnostic,
                               ModelInstanceIndex model_instance,
                               const sdf::Model& model,
                               MultibodyPlant<double>* plant,
                               CollisionFilterGroupResolver* resolver) {
  auto next_child_element = [](const ElementNode& data_element,
                               const char* element_name) {
    return std::get<sdf::ElementPtr>(data_element)
        ->GetElementImpl(std::string(element_name));
  };
  auto next_sibling_element = [](const ElementNode& data_element,
                                 const char* element_name) {
    return std::get<sdf::ElementPtr>(data_element)
        ->GetNextElement(std::string(element_name));
  };
  auto has_attribute = [](const ElementNode& data_element,
                          const char* attribute_name) {
    return std::get<sdf::ElementPtr>(data_element)
        ->HasAttribute(std::string(attribute_name));
  };
  auto get_string_attribute = [&diagnostic](
                                  const ElementNode& data_element,
                                  const char* attribute_name) -> std::string {
    auto element = std::get<sdf::ElementPtr>(data_element);
    if (!element->HasAttribute(attribute_name)) {
      std::string message =
          fmt::format("The tag <{}> is missing the required attribute \"{}\"",
                      element->GetName(), attribute_name);
      diagnostic.Error(element, std::move(message));
      return {};
    }
    return std::get<sdf::ElementPtr>(data_element)
        ->Get<std::string>(attribute_name);
  };
  auto get_bool_attribute = [](const ElementNode& data_element,
                               const char* attribute_name) {
    return std::get<sdf::ElementPtr>(data_element)->Get<bool>(attribute_name);
  };
  auto read_tag_string = [&diagnostic](const ElementNode& data_element,
                                       const char*) -> std::string {
    auto element = std::get<sdf::ElementPtr>(data_element);
    sdf::ParamPtr param = element->GetValue();
    if (param == nullptr) {
      std::string message =
          fmt::format("The tag <{}> is missing a required string value.",
                      element->GetName());
      diagnostic.Error(element, std::move(message));
      return {};
    }
    return param->GetAsString();
  };
  ParseCollisionFilterGroupCommon(
      diagnostic.MakePolicyForNode(*(model.Element())), model_instance,
      model.Element(), plant, resolver, next_child_element,
      next_sibling_element, has_attribute, get_string_attribute,
      get_bool_attribute, read_tag_string);
}

bool CanReuseModelInstance(
    const ModelInstanceIndex& model_instance,
    const ModelInstanceIndexRange reusable_model_instance_range) {
  return model_instance >= reusable_model_instance_range.first &&
         model_instance < reusable_model_instance_range.second;
}

ModelInstanceIndex AddModelInstanceIfReusable(
    MultibodyPlant<double>* plant, const std::string& model_name,
    const ModelInstanceIndexRange& reusable_model_instance_range) {
  if (plant->HasModelInstanceNamed(model_name)) {
    auto model_instance = plant->GetModelInstanceByName(model_name);
    if (!CanReuseModelInstance(model_instance, reusable_model_instance_range)) {
      throw std::logic_error(
          "This model already contains a model instance named '" + model_name +
          "'. Model instance names must be unique within a given model.");
    }
    return model_instance;
  } else {
    return plant->AddModelInstance(model_name);
  }
}

// Helper class that does bookkeeping on the frame bearing elements of a given
// model instance in a MultibodyPlant. This is used to keep track of newly
// created elements after a custom parser is called so that we can create
// Interface API data structures from just the newly created elements.
// The Interface API step is necessary so that we can construct the appropriate
// pose graph at higher levels of the hierarchy using libsdformat. Note: We
// assume that Drake custom parsers do not support nested models.
// TODO(azeey) If any of Drakes custom parsers start supporting nested models,
// this needs to be updated to keep track of them.
class InterfaceModelHelper {
 public:
  explicit InterfaceModelHelper(const MultibodyPlant<double>& pt)
      : plant_(pt) {}

  std::vector<ModelInstanceIndex> GetChildModelInstanceIndices() const {
    std::vector<ModelInstanceIndex> output;

    for (ModelInstanceIndex mi(model_instance_ + 1);
         mi < plant_.num_model_instances(); ++mi) {
      if (multibody::ScopedName::Parse(plant_.GetModelInstanceName(mi))
              .get_element() == name_) {
        output.push_back(mi);
      }
    }
    return output;
  }

  // Take a snapshot of the current contents of a model instance so that we
  // later on do a diff
  void TakeSnapShot(ModelInstanceIndex model_instance_index) {
    model_instance_ = model_instance_index;
    name_ = plant_.GetModelInstanceName(model_instance_);
    body_indices_ = plant_.GetBodyIndices(model_instance_);
    frame_indices_ = plant_.GetFrameIndices(model_instance_);
    joint_indices_ = plant_.GetJointIndices(model_instance_);
    model_instance_indices_ = GetChildModelInstanceIndices();
    have_snapshot = true;
  }

  template <typename T>
  static std::vector<T> GetVectorDiff(const std::vector<T>& current,
                                      const std::vector<T>& previous) {
    std::vector<T> output;
    std::set_difference(current.begin(), current.end(), previous.begin(),
                        previous.end(), std::back_inserter(output));
    return output;
  }

  // Compute the diff to determine the elements that were created after the
  // snapshot.
  void ComputeDiffFromSnapshot() {
    DRAKE_DEMAND(have_snapshot);
    body_indices_ =
        GetVectorDiff(plant_.GetBodyIndices(model_instance_), body_indices_);
    frame_indices_ =
        GetVectorDiff(plant_.GetFrameIndices(model_instance_), frame_indices_);
    joint_indices_ =
        GetVectorDiff(plant_.GetJointIndices(model_instance_), joint_indices_);
    model_instance_indices_ =
        GetVectorDiff(GetChildModelInstanceIndices(), model_instance_indices_);
  }

  ModelInstanceIndex model_instance() const { return model_instance_; }

  const std::vector<JointIndex>& joint_indices() const {
    return joint_indices_;
  }
  const std::vector<FrameIndex>& frame_indices() const {
    return frame_indices_;
  }
  const std::vector<BodyIndex>& body_indices() const { return body_indices_; }
  const std::string& model_frame_name() const { return model_frame_name_; }
  void set_model_frame_name(const std::string& model_frame_name) {
    model_frame_name_ = model_frame_name;
  }

 private:
  const MultibodyPlant<double>& plant_;
  ModelInstanceIndex model_instance_;
  std::string name_;
  std::vector<BodyIndex> body_indices_;
  std::vector<FrameIndex> frame_indices_;
  std::vector<JointIndex> joint_indices_;
  std::vector<ModelInstanceIndex> model_instance_indices_;
  std::string model_frame_name_ = "__model__";

  bool have_snapshot{false};
};

// Helper method to add a model to a MultibodyPlant given an sdf::Model
// specification object.
std::vector<ModelInstanceIndex> AddModelsFromSpecification(
    const SDFormatDiagnostic& diagnostic, sdf::Model* model_ptr,
    const std::string& model_name, const RigidTransformd& X_WP,
    MultibodyPlant<double>* plant, CollisionFilterGroupResolver* resolver,
    const PackageMap& package_map, const std::string& root_dir,
    const ModelInstanceIndexRange& reusable_model_instance_range,
    const sdf::ParserConfig& parser_config) {
  DRAKE_DEMAND(model_ptr != nullptr);

  DrakifyModel(diagnostic, parser_config, model_ptr);

  const sdf::Model& model = *model_ptr;
  const ModelInstanceIndex model_instance = AddModelInstanceIfReusable(
      plant, model_name, reusable_model_instance_range);

  const std::set<std::string> supported_model_elements{
      "drake:joint",
      "drake:linear_bushing_rpy",
      "drake:linear_spring_damper",
      "drake:ball_constraint",
      "drake:tendon_constraint",
      "drake:collision_filter_group",
      "frame",
      "include",
      "joint",
      "link",
      "model",
      "pose",
      "static"};
  CheckSupportedElements(diagnostic, model.Element(), supported_model_elements);

  std::vector<ModelInstanceIndex> added_model_instances{model_instance};

  // "P" is the parent frame. If the model is in a child of //world or //sdf,
  // this will be the world frame. Otherwise, this will be the parent model
  // frame.
  const RigidTransformd X_PM =
      ResolveRigidTransform(diagnostic, model.SemanticPose());
  const RigidTransformd X_WM = X_WP * X_PM;

  // Add nested models at root-level of <model>.
  // Do this before the resolving canonical link because the link might be in a
  // nested model.
  drake::log()->trace("sdf_parser: Add nested models");
  for (uint64_t model_index = 0; model_index < model.ModelCount();
       ++model_index) {
    sdf::Model* nested_model = model_ptr->ModelByIndex(model_index);
    std::vector<ModelInstanceIndex> nested_model_instances =
        AddModelsFromSpecification(
            diagnostic, nested_model,
            sdf::JoinName(model_name, nested_model->Name()), X_WM, plant,
            resolver, package_map, root_dir, reusable_model_instance_range,
            parser_config);

    added_model_instances.insert(added_model_instances.end(),
                                 nested_model_instances.begin(),
                                 nested_model_instances.end());
  }
  drake::log()->trace("sdf_parser: Add links");
  std::vector<LinkInfo> rigid_link_infos;

  // Add all the links
  for (uint64_t link_index = 0; link_index < model.LinkCount(); ++link_index) {
    const sdf::Link& link = *model.LinkByIndex(link_index);
    if (IsDeformableLink(link)) {
      if (!AddDeformableLinkFromSpecification(diagnostic, model_instance, link,
                                              X_WM, plant, package_map,
                                              root_dir)) {
        return {};
      }
    } else {
      std::optional<LinkInfo> link_info = AddRigidLinkFromSpecification(
          diagnostic, model_instance, link, X_WM, plant, package_map, root_dir);
      if (link_info.has_value()) {
        rigid_link_infos.push_back(*link_info);
      } else {
        // If we fail to add a link, we should not continue.
        return {};
      }
    }
  }

  // Add the SDF "model frame" given the model name so that way any frames
  // added to the plant are associated with this current model instance. N.B.
  // This follows SDFormat's convention.
  const std::string sdf_model_frame_name = "__model__";

  drake::log()->trace("sdf_parser: Resolve canonical link");
  const Frame<double>& model_frame = [&]() -> const Frame<double>& {
    const auto [canonical_link, canonical_link_name] =
        model.CanonicalLinkAndRelativeName();

    if (canonical_link != nullptr && !IsDeformableLink(*canonical_link)) {
      const auto [parent_model_instance, local_name] =
          GetResolvedModelInstanceAndLocalName(canonical_link_name,
                                               model_instance, *plant);
      const Frame<double>& canonical_link_frame =
          plant->GetFrameByName(local_name, parent_model_instance);
      const RigidTransformd X_LcM = ResolveRigidTransform(
          diagnostic, model.SemanticPose(),
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
    if (!AddJointFromSpecification(diagnostic, X_WM, joint, model_instance,
                                   plant, &joint_types)) {
      return {};
    }
  }

  // Parse drake:mimic elements only after all joints have been added.
  for (uint64_t joint_index = 0; joint_index < model.JointCount();
       ++joint_index) {
    // Get a pointer to the SDF joint, and the joint axis information.
    const sdf::Joint& joint = *model.JointByIndex(joint_index);
    if (!ParseMimicTag(diagnostic, joint, model_instance, plant)) {
      return {};
    }
  }

  drake::log()->trace("sdf_parser: Add explicit frames");
  // Add frames at root-level of <model>.
  for (uint64_t frame_index = 0; frame_index < model.FrameCount();
       ++frame_index) {
    const sdf::Frame& frame = *model.FrameByIndex(frame_index);
    AddFrameFromSpecification(diagnostic, frame, model_instance, model_frame,
                              plant);
  }

  drake::log()->trace("sdf_parser: Add drake custom joints");
  if (model.Element()->HasElement("drake:joint")) {
    for (sdf::ElementPtr joint_node =
             model.Element()->GetElement("drake:joint");
         joint_node; joint_node = joint_node->GetNextElement("drake:joint")) {
      if (!AddDrakeJointFromSpecification(diagnostic, joint_node,
                                          model_instance, plant)) {
        return {};
      }
    }
  }

  drake::log()->trace("sdf_parser: Add linear_bushing_rpy");
  if (model.Element()->HasElement("drake:linear_bushing_rpy")) {
    for (sdf::ElementPtr bushing_node =
             model.Element()->GetElement("drake:linear_bushing_rpy");
         bushing_node; bushing_node = bushing_node->GetNextElement(
                           "drake:linear_bushing_rpy")) {
      if (AddBushingFromSpecification(diagnostic, bushing_node, model_instance,
                                      plant) == nullptr) {
        return {};
      }
    }
  }

  drake::log()->trace("sdf_parser: Add linear_spring_damper");
  if (model.Element()->HasElement("drake:linear_spring_damper")) {
    for (sdf::ElementPtr linear_spring_damper_node =
             model.Element()->GetElement("drake:linear_spring_damper");
         linear_spring_damper_node;
         linear_spring_damper_node = linear_spring_damper_node->GetNextElement(
             "drake:linear_spring_damper")) {
      if (AddLinearSpringDamperFromSpecification(
              diagnostic, linear_spring_damper_node, model_instance, plant) ==
          nullptr) {
        return {};
      }
    }
  }

  drake::log()->trace("sdf_parser: Add BallConstraint");
  if (model.Element()->HasElement("drake:ball_constraint")) {
    for (sdf::ElementPtr constraint_node =
             model.Element()->GetElement("drake:ball_constraint");
         constraint_node; constraint_node = constraint_node->GetNextElement(
                              "drake:ball_constraint")) {
      AddBallConstraintFromSpecification(diagnostic, constraint_node,
                                         model_instance, plant);
    }
  }

  drake::log()->trace("sdf_parser: Add TendonConstraint");
  if (model.Element()->HasElement("drake:tendon_constraint")) {
    for (sdf::ElementPtr constraint_node =
             model.Element()->GetElement("drake:tendon_constraint");
         constraint_node; constraint_node = constraint_node->GetNextElement(
                              "drake:tendon_constraint")) {
      AddTendonConstraintFromSpecification(diagnostic, constraint_node,
                                           model_instance, plant);
    }
  }

  if (model.Static()) {
    // Only weld / fixed joints are permissible.
    // TODO(eric.cousineau): Consider "freezing" non-weld joints, as is
    // permissible in Bullet and DART via Gazebo (#12227).
    for (sdf::JointType joint_type : joint_types) {
      if (joint_type != sdf::JointType::FIXED) {
        std::string message =
            "Only fixed joints are permitted in static models.";
        diagnostic.Error(model.Element(), std::move(message));
        return {};
      }
    }
    // Weld all links that have been added, but are not (yet) attached to the
    // world.
    // N.B. This implementation complicates "reposturing" a static model after
    // parsing. See #12227 and #14518 for more discussion.
    for (const LinkInfo& link_info : rigid_link_infos) {
      if (!AreWelded(*plant, plant->world_body(), *link_info.body)) {
        const auto& A = plant->world_frame();
        const auto& B = link_info.body->body_frame();
        const std::string joint_name =
            "sdformat_model_static_" + A.name() + "_welds_to_" + B.name();
        plant->AddJoint(std::make_unique<WeldJoint<double>>(joint_name, A, B,
                                                            link_info.X_WL));
      }
    }
  }

  // Parses the collision filter groups only if the scene graph is registered.
  if (plant->geometry_source_is_registered()) {
    drake::log()->trace("sdf_parser: Add collision filter groups");
    ParseCollisionFilterGroup(diagnostic, model_instance, model, plant,
                              resolver);
  }

  return added_model_instances;
}

// Helper function that computes the default pose of a Frame
RigidTransformd GetDefaultFramePose(const MultibodyPlant<double>& plant,
                                    const Frame<double>& frame) {
  const RigidTransformd X_WB =
      plant.GetDefaultFloatingBaseBodyPose(frame.body());
  const RigidTransformd X_WF = X_WB * frame.GetFixedPoseInBodyFrame();
  return X_WF;
}

// For the libsdformat API, see:
// http://sdformat.org/tutorials?tut=composition_proposal
constexpr char kExtUrdf[] = ".urdf";
constexpr char kExtXml[] = ".xml";

void AddBodiesToInterfaceModel(const MultibodyPlant<double>& plant,
                               const sdf::InterfaceModelPtr& interface_model,
                               const std::vector<BodyIndex>& body_indices,
                               const RigidTransformd& X_MW) {
  for (auto index : body_indices) {
    const auto& link = plant.get_body(index);
    RigidTransformd X_ML = X_MW * plant.GetDefaultFloatingBaseBodyPose(link);
    interface_model->AddLink({link.name(), ToIgnitionPose3d(X_ML)});
  }
}

void AddFramesToInterfaceModel(const MultibodyPlant<double>& plant,
                               ModelInstanceIndex model_instance,
                               const sdf::InterfaceModelPtr& interface_model,
                               const std::vector<FrameIndex>& frame_indices,
                               const std::string& model_frame_name) {
  for (auto index : frame_indices) {
    const auto& frame = plant.get_frame(index);
    if (frame.model_instance() != model_instance) {
      continue;
    }
    if (frame.name().empty() || frame.name() == model_frame_name ||
        plant.HasBodyNamed(frame.name(), model_instance)) {
      // Skip unnamed frames, and __model__ since it's already added.  Also
      // skip frames with the same name as a link since those are frames added
      // by Drake and are considered implicit by SDFormat. Sending such frames
      // to SDFormat would imply that these frames are explicit (i.e., frames
      // created using the <frame> tag).
      continue;
    }
    interface_model->AddFrame(
        {frame.name(), GetRelativeBodyName(frame.body(), model_instance, plant),
         ToIgnitionPose3d(frame.GetFixedPoseInBodyFrame())});
  }
}

void AddJointsToInterfaceModel(const MultibodyPlant<double>& plant,
                               const sdf::InterfaceModelPtr& interface_model,
                               const std::vector<JointIndex>& joint_indices) {
  for (auto index : joint_indices) {
    const auto& joint = plant.get_joint(index);
    const std::string& child_name = joint.child_body().name();
    const RigidTransformd X_CJ =
        joint.frame_on_child().GetFixedPoseInBodyFrame();
    interface_model->AddJoint(
        {joint.name(), child_name, ToIgnitionPose3d(X_CJ)});
  }
}

// This is a forward-declaration of an anonymous helper that's defined later
// in this file.
sdf::ParserConfig MakeSdfParserConfig(const ParsingWorkspace&,
                                      const std::string& /* root_dir */);

sdf::Error MakeSdfError(sdf::ErrorCode code, const DiagnosticDetail& detail) {
  sdf::Error result(code, detail.message);
  if (detail.filename.has_value()) {
    result.SetFilePath(*detail.filename);
  }
  if (detail.line.has_value()) {
    result.SetLineNumber(*detail.line);
  }
  return result;
}

ModelInstanceIndex GetOrCreateModelInstanceByName(
    MultibodyPlant<double>* plant, const std::string& model_name) {
  if (plant->HasModelInstanceNamed(model_name)) {
    return plant->GetModelInstanceByName(model_name);
  }
  return plant->AddModelInstance(model_name);
}

sdf::InterfaceModelPtr ConvertToInterfaceModel(
    MultibodyPlant<double>* plant, std::string model_name,
    const InterfaceModelHelper& interface_model_helper, sdf::Errors* errors,
    RigidTransformd X_WP = RigidTransformd::Identity()) {
  auto model_instance = interface_model_helper.model_instance();
  sdf::RepostureFunction reposture_model =
      [plant, model_instance,
       errors](const sdf::InterfaceModelPoseGraph& graph) {
        // N.B. This should also posture the model appropriately.
        for (auto interface_link_ind : plant->GetBodyIndices(model_instance)) {
          const auto& interface_link = plant->get_body(interface_link_ind);

          gz::math::Pose3d X_WL;
          sdf::Errors inner_errors =
              graph.ResolveNestedFramePose(X_WL, interface_link.name());
          PropagateErrors(std::move(inner_errors), errors);
          plant->SetDefaultFloatingBaseBodyPose(interface_link,
                                                ToRigidTransform(X_WL));
        }
      };

  const auto& model_frame = plant->GetFrameByName(
      interface_model_helper.model_frame_name(), model_instance);
  const std::string canonical_link_name =
      GetRelativeBodyName(model_frame.body(), model_instance, *plant);
  const RigidTransformd X_WM = GetDefaultFramePose(*plant, model_frame);
  const RigidTransformd X_PM = X_WP.inverse() * X_WM;

  auto interface_model = std::make_shared<sdf::InterfaceModel>(
      model_name, reposture_model, false, canonical_link_name,
      ToIgnitionPose3d(X_PM));

  AddBodiesToInterfaceModel(*plant, interface_model,
                            interface_model_helper.body_indices(),
                            X_WM.inverse());

  AddFramesToInterfaceModel(*plant, model_instance, interface_model,
                            interface_model_helper.frame_indices(),
                            interface_model_helper.model_frame_name());

  AddJointsToInterfaceModel(*plant, interface_model,
                            interface_model_helper.joint_indices());

  return interface_model;
}

// This assumes that parent models will have their parsing start before child
// models! This is a safe assumption because we only encounter nested models
// when force testing SDFormat files and libsdformat parses models in a top-down
// order. If we add support for other file formats, we should ensure that the
// parsers comply with this assumption.
sdf::InterfaceModelPtr ParseNestedInterfaceModel(
    const ParsingWorkspace& workspace, const sdf::NestedInclude& include,
    sdf::Errors* errors, const std::string& root_dir) {
  const sdf::ParserConfig parser_config =
      MakeSdfParserConfig(workspace, root_dir);
  auto& [options, package_map, diagnostic, builder, plant, scene_graph,
         collision_resolver, parser_selector] = workspace;
  const std::string resolved_filename{include.ResolvedFileName()};

  // Do not attempt to parse anything other than URDF and MuJoCo xml files.
  const bool is_urdf = EndsWithCaseInsensitive(resolved_filename, kExtUrdf);
  const bool is_xml = EndsWithCaseInsensitive(resolved_filename, kExtXml);
  if (!is_urdf && !is_xml) {
    return nullptr;
  }

  if (include.IsStatic()) {
    errors->emplace_back(
        sdf::ErrorCode::ELEMENT_INVALID,
        "Drake does not yet support //include/static for custom nesting.");
    return nullptr;
  }

  DataSource data_source(DataSource::kFilename, &resolved_filename);
  drake::internal::DiagnosticPolicy subdiagnostic;
  subdiagnostic.SetActionForWarnings([&errors](const DiagnosticDetail& detail) {
    errors->emplace_back(MakeSdfError(sdf::ErrorCode::NONE, detail));
  });
  subdiagnostic.SetActionForErrors([&errors](const DiagnosticDetail& detail) {
    errors->emplace_back(MakeSdfError(sdf::ErrorCode::ELEMENT_INVALID, detail));
  });

  ModelInstanceIndex main_model_instance;
  // New instances will have indices starting from cur_num_models
  const bool is_merge_include = include.IsMerge().value_or(false);

  InterfaceModelHelper interface_model_helper(*plant);
  ParsingWorkspace subworkspace{options,        package_map, subdiagnostic,
                                builder,        plant,       collision_resolver,
                                parser_selector};

  std::string model_frame_name = "__model__";
  std::string model_name;
  if (is_merge_include) {
    // Create the parent model instance if it hasn't been created already.
    // This can happen if this is the first model to be merge-included.
    const auto parent_model_instance =
        GetOrCreateModelInstanceByName(plant, include.AbsoluteParentName());

    interface_model_helper.TakeSnapShot(parent_model_instance);
    auto& parser = parser_selector(diagnostic, resolved_filename);
    model_name =
        parser.MergeModel(data_source, include.LocalModelName().value_or(""),
                          parent_model_instance, subworkspace);
    interface_model_helper.ComputeDiffFromSnapshot();

    model_frame_name = sdf::computeMergedModelProxyFrameName(model_name);
    interface_model_helper.set_model_frame_name(model_frame_name);

    main_model_instance = parent_model_instance;
  } else {
    const std::optional<ModelInstanceIndex> maybe_model =
        parser_selector(diagnostic, resolved_filename)
            .AddModel(data_source, include.LocalModelName().value_or(""),
                      include.AbsoluteParentName(), subworkspace);
    if (maybe_model.has_value()) {
      main_model_instance = *maybe_model;
    } else {
      return nullptr;
    }

    model_name =
        ScopedName::Parse(plant->GetModelInstanceName(main_model_instance))
            .get_element();

    // In the non-merge-include case, we don't need to compute the diff, but we
    // still use the interface_model_helper to hold all the frame bearing
    // elements of the newly created model instance so we can later convert
    // this model instance to an Interface Model for libsdformat's Interface
    // API.
    interface_model_helper.TakeSnapShot(main_model_instance);
  }
  DRAKE_DEMAND(!model_name.empty());

  // Add explicit model frame to first link.
  auto body_indices = workspace.plant->GetBodyIndices(main_model_instance);
  if (body_indices.empty()) {
    errors->emplace_back(sdf::ErrorCode::ELEMENT_INVALID,
                         "Model must have at least one link.");
    return nullptr;
  }
  const auto& canonical_link = workspace.plant->get_body(body_indices[0]);

  const Frame<double>& canonical_link_frame =
      plant->GetFrameByName(canonical_link.name(), main_model_instance);
  plant->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
      model_frame_name, canonical_link_frame, RigidTransformd::Identity(),
      main_model_instance));

  // Now that the model is parsed, we create interface elements to send to
  // libsdformat.

  // This will be populated for the first model instance.
  sdf::InterfaceModelPtr main_interface_model;
  main_interface_model = ConvertToInterfaceModel(
      plant, model_name, interface_model_helper, errors);
  main_interface_model->SetParserSupportsMergeInclude(true);

  return main_interface_model;
}

// Convert a Drake SpatialInertia into a gazebo Inertia1d. No frames are noted
// because the conversion doesn't depend on the frames -- the same frame
// interpretation applies to the output and input.
gz::math::Inertiald MakeGzInertia(const SpatialInertia<double>& M) {
  const Vector3d& p_cm = M.get_com();
  const double mass = M.get_mass();
  const RotationalInertia<double>& I = M.CalcRotationalInertia();
  const Vector3d moments = I.get_moments();
  const Vector3d products = I.get_products();
  const RollPitchYawd rpy(0, 0, 0);
  return gz::math::Inertiald(
      gz::math::MassMatrix3d(
          mass, gz::math::Vector3(moments.x(), moments.y(), moments.z()),
          gz::math::Vector3(products.x(), products.y(), products.z())),
      gz::math::Pose3d(p_cm.x(), p_cm.y(), p_cm.z(), rpy.roll_angle(),
                       rpy.pitch_angle(), rpy.yaw_angle()));
}

gz::math::Inertiald CalculateMeshInertia(const ParsingWorkspace& workspace,
                                         double density, const sdf::Mesh& mesh,
                                         const std::string& root_dir) {
  if (!(std::isfinite(density) && density >= 0.0)) {
    std::optional<int> line_number_maybe = mesh.Element()->LineNumber();
    workspace.diagnostic.Error(fmt::format(
        "The <mesh> tag{} specifies a non-physical density value: {} m/kg³. "
        "Density must be a positive, finite value.",
        line_number_maybe ? fmt::format(" on line {}", *line_number_maybe)
                          : std::string(),
        density));
    return gz::math::Inertiald();
  }
  // If the sdf hasn't specified a mesh URI, SDFormat provides "__default__".
  if (mesh.Uri() == "__default__") {
    // No error logged; the error will get logged when we actually attempt to
    // *make* the geometry::Mesh.
    return gz::math::Inertiald();
  }

  // TODO(SeanCurtis-TRI): The sdformat API for this callback includes a
  // CustomInertiaCalcProperties struct. We're not using it so it is omitted
  // here. We could conceivably use it to give the user more control over how
  // the spatial inertia is computed. E.g., fail instead of fallback to Convex,
  // or only use Convex in the first place, etc.
  // https://github.com/gazebosim/sdformat/blob/main/include/sdf/CustomInertiaCalcProperties.hh
  //
  // ...
  // <link name="link">
  //   <inertial auto="true">
  //     <auto_inertia_params>
  //       <drake:force_convex_hull>true</drake:force_convex_hull>
  //     </auto_inertia_params>
  //   <inertial>
  // </link>
  // ...

  auto resolve_filename = [&workspace, &root_dir](
                              const DiagnosticPolicy& diagnostic,
                              const std::string& uri_local) {
    const ResolveUriResult resolved =
        ResolveUri(diagnostic, uri_local, workspace.package_map, root_dir);
    return resolved.GetStringPathIfExists();
  };

  const std::string filename =
      resolve_filename(workspace.diagnostic, mesh.Uri());
  const Vector3d scale = ToVector3(mesh.Scale());
  const geometry::Mesh mesh_geo(filename, scale);

  CalcSpatialInertiaResult result = internal::CalcSpatialInertiaWithFallback(
      mesh_geo, density,
      /* warn_for_convex= */ [&workspace](const std::string& message) {
        workspace.diagnostic.Warning(message);
      });

  if (std::holds_alternative<std::string>(result)) {
    workspace.diagnostic.Error(fmt::format(
        "Failed to compute spatial inertia even for the convex hull of "
        "{}.\n{}",
        mesh_geo.source().path().string(), std::get<std::string>(result)));
    return gz::math::Inertiald();
  }

  const SpatialInertia<double>& M_GGo_G =
      std::get<SpatialInertia<double>>(result);

  return MakeGzInertia(M_GGo_G);
}

// TODO(10218) As per the issue, we eventually want to disallow relative-path
// files (preferring package:// urls). But, for now, we maintain backwards
// compatibility with the existing behavior. When resolving the issue, we'll be
// able to eliminate the `root_dir` argument.
// Note that this function keeps an alias of the inputs `workspace` and
// `root_dir` in its return value. Therefore, the lifetime of the inputs must be
// greater than that of the returned parser config object.
sdf::ParserConfig MakeSdfParserConfig(const ParsingWorkspace& workspace,
                                      const std::string& root_dir) {
  // The error severity settings here are somewhat subtle. We set all of them
  // to ERR so that reports will append into an sdf::Errors collection instead
  // of spamming into spdlog. However, when grabbing the reports out of the
  // sdf::Errors collection later, we will categorize certain kinds of reports
  // (e.g., deprecations) back to a DiagnosticPolicy::Warning instead of Error.
  // That filter is specified in our IsError() helper, above.
  sdf::ParserConfig parser_config;
  parser_config.SetWarningsPolicy(sdf::EnforcementPolicy::ERR);
  parser_config.SetDeprecatedElementsPolicy(sdf::EnforcementPolicy::ERR);
  parser_config.SetUnrecognizedElementsPolicy(sdf::EnforcementPolicy::ERR);
  parser_config.SetFindCallback([&workspace](const std::string& _input) {
    // This callback uses an empty return value to denote errors, and then its
    // caller reports its own "no such file" error directly. We'll route
    // Drake's specific messages about *why* the file wasn't found into a
    // debug-only log.
    DiagnosticPolicy debug_log;
    debug_log.SetActionForWarnings([](const DiagnosticDetail& detail) {
      drake::log()->debug(detail.FormatWarning());
    });
    debug_log.SetActionForErrors([](const DiagnosticDetail& detail) {
      drake::log()->debug(detail.FormatError());
    });
    const ResolveUriResult resolved =
        ResolveUri(debug_log, _input, workspace.package_map, ".");
    return resolved.GetStringPathIfExists();
  });

  parser_config.RegisterCustomModelParser(
      [&workspace, &root_dir](const sdf::NestedInclude& include,
                              sdf::Errors& errors) {
        return ParseNestedInterfaceModel(workspace, include, &errors, root_dir);
      });

  auto mesh_inertia_calculator =
      [&workspace, &root_dir](
          sdf::Errors&, const sdf::CustomInertiaCalcProperties& properties)
      -> std::optional<gz::math::Inertiald> {
    const std::optional<sdf::Mesh>& maybe_mesh = properties.Mesh();
    // This callback is only invoked from within sdf::Mesh (in which the mesh
    // passes *itself* into the `properties`). As such, maybe_mesh should always
    // have a value.
    DRAKE_DEMAND(maybe_mesh.has_value());
    return CalculateMeshInertia(workspace, properties.Density(), *maybe_mesh,
                                root_dir);
  };
  parser_config.RegisterCustomInertiaCalc(mesh_inertia_calculator);

  return parser_config;
}

sdf::Model* get_only_model(sdf::Root* root) {
  sdf::Model* maybe_model = root->Model();
  if (maybe_model != nullptr) {
    return maybe_model;
  }
  // If Model() is null, there still may be a single world with a single model
  // (ignoring nested models). Try to find that.
  if (root->WorldCount() != 1) {
    return nullptr;
  }
  sdf::World* world0 = root->WorldByIndex(0);
  if (world0->ModelCount() != 1) {
    return nullptr;
  }
  return world0->ModelByIndex(0);
}

}  // namespace

std::optional<ModelInstanceIndex> AddModelFromSdf(
    const DataSource& data_source, const std::string& model_name_in,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  DRAKE_THROW_UNLESS(!workspace.plant->is_finalized());

  sdf::ParserConfig parser_config =
      MakeSdfParserConfig(workspace, data_source.GetRootDir());

  sdf::Root root;

  SDFormatDiagnostic diagnostic(&workspace.diagnostic, &data_source);

  const auto model_index_begin =
      static_cast<ModelInstanceIndex>(workspace.plant->num_model_instances());
  sdf::Errors errors = LoadSdf(diagnostic, &root, data_source, parser_config);
  if (diagnostic.PropagateErrors(errors)) {
    return std::nullopt;
  }
  const auto model_index_end =
      static_cast<ModelInstanceIndex>(workspace.plant->num_model_instances());

  ModelInstanceIndexRange reusable_model_instance_range =
      std::make_pair(model_index_begin, model_index_end);

  sdf::Model* model_ptr = get_only_model(&root);
  if (model_ptr == nullptr) {
    std::string message = "File must have a single <model> element.";
    diagnostic.Error(root.Element(), std::move(message));
    return std::nullopt;
  }

  const std::string local_model_name =
      model_name_in.empty() ? model_ptr->Name() : model_name_in;

  std::string model_name =
      MakeModelName(local_model_name, parent_model_name, workspace);

  std::vector<ModelInstanceIndex> added_model_instances =
      AddModelsFromSpecification(diagnostic, model_ptr, model_name, {},
                                 workspace.plant, workspace.collision_resolver,
                                 workspace.package_map,
                                 data_source.GetRootDir(),
                                 reusable_model_instance_range, parser_config);

  DRAKE_DEMAND(!added_model_instances.empty());
  return added_model_instances.front();
}

std::vector<ModelInstanceIndex> AddModelsFromSdf(
    const DataSource& data_source,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  DRAKE_THROW_UNLESS(!workspace.plant->is_finalized());

  sdf::ParserConfig parser_config =
      MakeSdfParserConfig(workspace, data_source.GetRootDir());

  sdf::Root root;

  SDFormatDiagnostic diagnostic(&workspace.diagnostic, &data_source);

  const auto model_index_begin =
      static_cast<ModelInstanceIndex>(workspace.plant->num_model_instances());
  sdf::Errors errors = LoadSdf(diagnostic, &root, data_source, parser_config);
  if (diagnostic.PropagateErrors(errors)) {
    return {};
  }
  const auto model_index_end =
      static_cast<ModelInstanceIndex>(workspace.plant->num_model_instances());

  // See comment on where the ModelInstanceIndexRange type is defined.
  ModelInstanceIndexRange reusable_model_instance_range =
      std::make_pair(model_index_begin, model_index_end);

  // There either must be exactly one model, or exactly one world.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const uint64_t model_count = root.Model() != nullptr ? 1 : 0;
#pragma GCC diagnostic pop
  const uint64_t world_count = root.WorldCount();
  if ((model_count + world_count) != 1) {
    std::string message = fmt::format(
        "File must have exactly one <model> or exactly one <world>, but"
        " instead has {} models and {} worlds",
        model_count, world_count);
    diagnostic.Error(root.Element(), std::move(message));
    return {};
  }

  std::vector<ModelInstanceIndex> model_instances;

  // At this point there should be only Models or a single World at the Root
  // level.
  if (model_count > 0) {
    DRAKE_DEMAND(model_count == 1);
    DRAKE_DEMAND(world_count == 0);
    DRAKE_DEMAND(root.Model() != nullptr);
    sdf::Model* model_ptr = root.Model();
    DRAKE_DEMAND(model_ptr != nullptr);

    std::string model_name =
        MakeModelName(model_ptr->Name(), parent_model_name, workspace);

    std::vector<ModelInstanceIndex> added_model_instances =
        AddModelsFromSpecification(
            diagnostic, model_ptr, model_name, {}, workspace.plant,
            workspace.collision_resolver, workspace.package_map,
            data_source.GetRootDir(), reusable_model_instance_range,
            parser_config);
    model_instances.insert(model_instances.end(), added_model_instances.begin(),
                           added_model_instances.end());
  } else {
    DRAKE_DEMAND(model_count == 0);
    DRAKE_DEMAND(world_count == 1);
    DRAKE_DEMAND(root.WorldByIndex(0) != nullptr);
    sdf::World& world = *root.WorldByIndex(0);

    const std::set<std::string> supported_world_elements{"frame", "include",
                                                         "joint", "model"};
    CheckSupportedElements(diagnostic, world.Element(),
                           supported_world_elements);

    // TODO(eric.cousineau): Either support or explicitly prevent adding joints
    // via `//world/joint`, per this Bitbucket comment: https://bit.ly/2udQxhp

    for (uint64_t model_index = 0; model_index < world.ModelCount();
         ++model_index) {
      // Get the model.
      sdf::Model* model_ptr = world.ModelByIndex(model_index);
      DRAKE_DEMAND(model_ptr != nullptr);

      std::string model_name =
          MakeModelName(model_ptr->Name(), parent_model_name, workspace);

      std::vector<ModelInstanceIndex> added_model_instances =
          AddModelsFromSpecification(
              diagnostic, model_ptr, model_name, {}, workspace.plant,
              workspace.collision_resolver, workspace.package_map,
              data_source.GetRootDir(), reusable_model_instance_range,
              parser_config);
      model_instances.insert(model_instances.end(),
                             added_model_instances.begin(),
                             added_model_instances.end());
    }

    for (uint64_t frame_index = 0; frame_index < world.FrameCount();
         ++frame_index) {
      const sdf::Frame& frame = *world.FrameByIndex(frame_index);
      AddFrameFromSpecification(diagnostic, frame, world_model_instance(),
                                workspace.plant->world_frame(),
                                workspace.plant);
    }

    // Add all the joints
    std::set<sdf::JointType> joint_types;
    for (uint64_t joint_index = 0; joint_index < world.JointCount();
         ++joint_index) {
      const sdf::Joint& joint = *world.JointByIndex(joint_index);
      if (!AddJointFromSpecification(diagnostic, {}, joint,
                                     world_model_instance(), workspace.plant,
                                     &joint_types, false)) {
        return {};
      }
    }

    // Parse drake:mimic elements only after all joints have been added.
    for (uint64_t joint_index = 0; joint_index < world.JointCount();
         ++joint_index) {
      // Get a pointer to the SDF joint, and the joint axis information.
      const sdf::Joint& joint = *world.JointByIndex(joint_index);
      if (!ParseMimicTag(diagnostic, joint, world_model_instance(),
                         workspace.plant)) {
        return {};
      }
    }

    for (sdf::JointType joint_type : joint_types) {
      if (joint_type != sdf::JointType::FIXED) {
        std::string message =
            "Only fixed joints are permitted in world joints.";
        diagnostic.Error(world.Element(), std::move(message));
        return {};
      }
    }
  }

  return model_instances;
}

SdfParserWrapper::SdfParserWrapper() {}

SdfParserWrapper::~SdfParserWrapper() {}

std::optional<ModelInstanceIndex> SdfParserWrapper::AddModel(
    const DataSource& data_source, const std::string& model_name,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  return AddModelFromSdf(data_source, model_name, parent_model_name, workspace);
}

std::vector<ModelInstanceIndex> SdfParserWrapper::AddAllModels(
    const DataSource& data_source,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  return AddModelsFromSdf(data_source, parent_model_name, workspace);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
