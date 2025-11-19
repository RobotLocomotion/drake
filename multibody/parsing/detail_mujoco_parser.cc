#include "drake/multibody/parsing/detail_mujoco_parser.h"

#include <algorithm>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <tinyxml2.h>

#include "drake/common/text_logging.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/shape_specification.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/detail_make_model_name.h"
#include "drake/multibody/parsing/detail_tinyxml.h"
#include "drake/multibody/parsing/detail_tinyxml2_diagnostic.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/geometry_spatial_inertia.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/scoped_name.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/systems/sensors/camera_config_functions.h"

namespace drake {
namespace multibody {
namespace internal {

using drake::internal::DiagnosticPolicy;
using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using geometry::GeometryId;
using geometry::GeometryInstance;
using math::RigidTransformd;
using math::RotationMatrixd;
using tinyxml2::XMLAttribute;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;
using tinyxml2::XMLNode;

namespace {

constexpr std::array kOrientationAttributes = {  // BR
    "quat", "axisangle", "euler", "xyaxes", "zaxis"};

class MujocoParser {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MujocoParser)

  explicit MujocoParser(const ParsingWorkspace& workspace,
                        const DataSource& data_source)
      : workspace_(workspace),
        diagnostic_(&workspace.diagnostic, &data_source),
        builder_(workspace.builder),
        plant_(workspace.plant),
        scene_graph_(workspace.scene_graph) {}

  void ErrorIfMoreThanOneOrientation(const XMLElement& node) {
    int num_orientation_attrs = 0;
    for (const char* attr : kOrientationAttributes) {
      if (node.Attribute(attr) != nullptr) {
        ++num_orientation_attrs;
      }
    }
    if (num_orientation_attrs > 1) {
      std::string attributes;
      for (const XMLAttribute* attr = node.FirstAttribute(); attr;
           attr = attr->Next()) {
        attributes += fmt::format("{}={} ", attr->Name(),
                                  fmt_debug_string(attr->Value()));
      }
      Error(node,
            fmt::format(
                "Element {} has more than one orientation attribute specified. "
                "There must be no more than one instance of `quat`, "
                "`axisangle`, `euler`, `xyaxes`, or `zaxis`. "
                "Attributes: [{}]",
                node.Name(), attributes));
    }
  }

  // Any attributes from `default` that are not specified in `node` will be
  // added to `node`. For orientation attributes, we do not apply orientation
  // attribute defaults if the node already has an orientation attribute.
  void ApplyDefaultAttributes(const std::string& element_name,
                              const std::string& class_name, XMLElement* node) {
    const std::pair<std::string, std::string> key{element_name, class_name};
    if (!defaults_.contains(key)) {
      return;
    }
    const XMLElement* default_node = defaults_.at(key);

    bool node_has_orientation_attr = false;
    for (const char* attr : kOrientationAttributes) {
      if (node->Attribute(attr) != nullptr) {
        node_has_orientation_attr = true;
        break;
      }
    }

    for (const XMLAttribute* default_attr = default_node->FirstAttribute();
         default_attr != nullptr; default_attr = default_attr->Next()) {
      if (!node->Attribute(default_attr->Name())) {
        if (node_has_orientation_attr &&
            std::find_if(kOrientationAttributes.cbegin(),
                         kOrientationAttributes.cend(),
                         [&default_attr](const char* attr) {
                           return strcmp(attr, default_attr->Name()) == 0;
                         }) != kOrientationAttributes.cend()) {
          // Don't apply default orientation attributes if the node already has
          // an orientation attribute.
          continue;
        }
        node->SetAttribute(default_attr->Name(), default_attr->Value());
      }
    }
  }

  RigidTransformd ParseTransform(
      XMLElement* node, const RigidTransformd& X_default = RigidTransformd{}) {
    Vector3d pos(X_default.translation());
    ParseVectorAttribute(node, "pos", &pos);
    ErrorIfMoreThanOneOrientation(*node);

    if (node->Attribute("quat") != nullptr) {
      Vector4d quat;  // MuJoCo uses w,x,y,z order.
      if (ParseVectorAttribute(node, "quat", &quat)) {
        return RigidTransformd(
            Eigen::Quaternion<double>(quat[0], quat[1], quat[2], quat[3]), pos);
      }
    }

    Vector4d axisangle;
    if (ParseVectorAttribute(node, "axisangle", &axisangle)) {
      if (angle_ == kDegree) {
        axisangle[3] *= (M_PI / 180.0);
      }
      return RigidTransformd(AngleAxisd(axisangle[3], axisangle.head<3>()),
                             pos);
    }

    Vector3d euler;
    if (ParseVectorAttribute(node, "euler", &euler)) {
      if (angle_ == kDegree) {
        euler *= (M_PI / 180.0);
      }
      Quaternion<double> quat(1, 0, 0, 0);
      DRAKE_DEMAND(eulerseq_.size() == 3);
      for (int i = 0; i < 3; ++i) {
        Quaternion<double> this_quat(cos(euler[i] / 2.0), 0, 0, 0);
        double sa = sin(euler[i] / 2.0);

        if (eulerseq_[i] == 'x' || eulerseq_[i] == 'X') {
          this_quat.x() = sa;
        } else if (eulerseq_[i] == 'y' || eulerseq_[i] == 'Y') {
          this_quat.y() = sa;
        } else {
          // We already confirmed that eulerseq_ only has 'xyzXYZ' when it was
          // parsed.
          DRAKE_DEMAND(eulerseq_[i] == 'z' || eulerseq_[i] == 'Z');
          this_quat.z() = sa;
        }

        if (eulerseq_[i] == 'x' || eulerseq_[i] == 'y' || eulerseq_[i] == 'z') {
          // moving axes: post-multiply
          quat = quat * this_quat;
        } else {
          // fixed axes: pre-multiply
          quat = this_quat * quat;
        }
      }
      return RigidTransformd(RotationMatrixd(quat), pos);
    }

    Vector6d xyaxes;
    if (ParseVectorAttribute(node, "xyaxes", &xyaxes)) {
      if (xyaxes.head<3>().norm() < 1e-12) {
        Error(*node,
              fmt::format(
                  "The x axis in the 'xyaxes' attribute '{}' is too small.",
                  node->Attribute("xyaxes")));
        return {};
      }
      if (xyaxes.tail<3>().norm() < 1e-12) {
        Error(*node,
              fmt::format(
                  "The y axis in the 'xyaxes' attribute '{}' is too small.",
                  node->Attribute("xyaxes")));
        return {};
      }
      Matrix3d R;
      // Normalize the x axis.
      R.col(0) = xyaxes.head<3>().normalized();
      // Make the y axis orthogonal to the x axis (and normalize).
      double d = R.col(0).dot(xyaxes.tail<3>());
      R.col(1) = (xyaxes.tail<3>() - d * R.col(0)).normalized();
      // Make the z axis orthogonal to the x and y axes (and normalize).
      R.col(2) = R.col(0).cross(R.col(1)).normalized();
      return RigidTransformd(RotationMatrixd(R), pos);
    }

    Vector3d zaxis;
    if (ParseVectorAttribute(node, "zaxis", &zaxis)) {
      // Minimal rotation that maps the vector (0,0,1) into the vector specified
      // here.
      return RigidTransformd(
          Eigen::Quaternion<double>::FromTwoVectors(Vector3d{0, 0, 1}, zaxis),
          pos);
    }

    return RigidTransformd(X_default.rotation(), pos);
  }

  // Returns true if limits were parsed.
  bool ParseLimits(XMLElement* node, const char* range_attr_name,
                   const char* limited_attr_name, Vector2d* limits) {
    std::string limited_attr;
    if (!ParseStringAttribute(node, limited_attr_name, &limited_attr)) {
      limited_attr = "auto";
    }

    bool has_range = ParseVectorAttribute(node, range_attr_name, limits);

    bool limited{false};
    if (limited_attr == "true") {
      limited = true;
    } else if (limited_attr == "false") {
      limited = false;
    } else if (limited_attr == "auto") {
      if (autolimits_) {
        limited = has_range;
      } else if (has_range) {
        // From the mujoco docs: In this mode [autolimits == false], it is an
        // error to specify a range without a limit.
        Error(*node, fmt::format("The '{}' attribute was specified but "
                                 "'autolimits' is disabled.",
                                 range_attr_name));
      }
    } else {
      Error(*node,
            fmt::format("The '{}' attribute must be one of 'true', 'false', "
                        "or 'auto'.",
                        limited_attr_name));
    }
    return limited;
  }

  void ParseMotorOrPosition(XMLElement* node) {
    std::string name;
    if (!ParseStringAttribute(node, "name", &name)) {
      // Use "motor#" as the default actuator name.
      name = fmt::format("motor{}", plant_->num_actuators());
    }

    std::string class_name;
    if (!ParseStringAttribute(node, "class", &class_name)) {
      class_name = "main";
    }
    ApplyDefaultAttributes(node->Name(), class_name, node);

    std::string joint_name;
    if (!ParseStringAttribute(node, "joint", &joint_name)) {
      Warning(*node, fmt::format(
                         "The motor '{}' does not use the 'joint' transmission "
                         "specification. Currently only the 'joint' attribute "
                         "is supported.  This motor will be ignored.",
                         name));
      return;
    }

    // Parse effort limits. For a motor, force limits are the same as control
    // limits, so we take the min of the two.
    double effort_limit = std::numeric_limits<double>::infinity();
    Vector2d ctrl_range(0.0, 0.0), force_range(0.0, 0.0);
    bool ctrl_limited =
        ParseLimits(node, "ctrlrange", "ctrllimited", &ctrl_range);
    bool force_limited =
        ParseLimits(node, "forcerange", "forcelimited", &force_range);

    if (ctrl_limited) {
      if (ctrl_range[0] > ctrl_range[1]) {
        Warning(
            *node,
            fmt::format(
                "The motor '{}' specified a ctrlrange attribute where lower "
                "limit > upper limit; these limits will be ignored.",
                name));
      } else {
        effort_limit = std::max(ctrl_range[1], -ctrl_range[0]);
        if (-ctrl_range[0] != ctrl_range[1]) {
          Warning(*node,
                  fmt::format("The motor '{}' specified a ctrlrange attribute "
                              "where lower limit != -upper limit. Asymmetrical "
                              "effort limits are not supported yet, so the "
                              "larger of the values {} will be used.",
                              name, effort_limit));
        }
      }
    }
    if (force_limited) {
      // For a motor, force limits are the same as control limits, so we take
      // the min of the two.
      if (force_range[0] > force_range[1]) {
        Warning(
            *node,
            fmt::format(
                "The motor '{}' specified a forcerange attribute where lower "
                "limit > upper limit; these limits will be ignored.",
                name));
      } else {
        effort_limit =
            std::min(effort_limit, std::max(force_range[1], -force_range[0]));
        if (-force_range[0] != force_range[1]) {
          Warning(*node,
                  fmt::format("The motor '{}' specified a forcerange attribute "
                              "where lower limit != -upper limit. Asymmetrical "
                              "effort limits are not supported yet, so the "
                              "larger of the values {} will be used.",
                              name, std::max(force_range[1], -force_range[0])));
        }
      }
    }

    // Unsupported attributes are listed in the order from the MuJoCo docs:
    // https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-motor
    WarnUnsupportedAttribute(*node, "group");

    WarnUnsupportedAttribute(*node, "lengthrange");
    WarnUnsupportedAttribute(*node, "cranklength");
    WarnUnsupportedAttribute(*node, "jointinparent");
    WarnUnsupportedAttribute(*node, "tendon");
    WarnUnsupportedAttribute(*node, "cranksite");
    WarnUnsupportedAttribute(*node, "slidersite");
    WarnUnsupportedAttribute(*node, "site");
    WarnUnsupportedAttribute(*node, "refsite");
    WarnUnsupportedAttribute(*node, "user");

    WarnUnsupportedAttribute(*node, "dyntype");
    WarnUnsupportedAttribute(*node, "gaintype");
    WarnUnsupportedAttribute(*node, "biastype");
    WarnUnsupportedAttribute(*node, "dynprm");
    WarnUnsupportedAttribute(*node, "gainprm");
    WarnUnsupportedAttribute(*node, "biasprm");

    const JointActuator<double>& actuator = plant_->AddJointActuator(
        name, plant_->GetJointByName(joint_name, model_instance_),
        effort_limit);

    const char* gear_attr = node->Attribute("gear");
    if (gear_attr) {
      std::vector<double> vals = ConvertToVector<double>(gear_attr);
      if (vals.size() != 1 && vals.size() != 6) {
        Warning(*node, fmt::format("Expected either 1 value or 6 values for "
                                   "'gear' attribute, but got {}.",
                                   gear_attr));
      }
      if (vals.size() > 0) {
        // Per the MuJoCo documentation: "For actuators with scalar
        // transmission, only the first element of this vector is used."
        plant_->get_mutable_joint_actuator(actuator.index())
            .set_default_gear_ratio(vals[0]);
      }
    }

    if (armature_.contains(actuator.joint().index())) {
      const double gear_ratio =
          plant_->get_joint_actuator(actuator.index()).default_gear_ratio();
      plant_->get_mutable_joint_actuator(actuator.index())
          .set_default_rotor_inertia(armature_.at(actuator.joint().index()) /
                                     (gear_ratio * gear_ratio));
    }

    if (std::string_view(node->Name()) == "position") {
      multibody::PdControllerGains gains{.p = 1, .d = 0};  // MuJoCo defaults.
      double kp, kd;
      if (ParseScalarAttribute(node, "kp", &kp)) {
        gains.p = kp;
      }
      if (ParseScalarAttribute(node, "kd", &kd)) {
        gains.d = kd;
      }
      plant_->get_mutable_joint_actuator(actuator.index())
          .set_controller_gains(gains);

      WarnUnsupportedAttribute(*node, "dampratio");
      WarnUnsupportedAttribute(*node, "timeconst");
      WarnUnsupportedAttribute(*node, "inheritrange");
    }
  }

  void ParseActuator(XMLElement* node) {
    for (XMLElement* motor_node = node->FirstChildElement("motor"); motor_node;
         motor_node = motor_node->NextSiblingElement("motor")) {
      ParseMotorOrPosition(motor_node);
    }
    for (XMLElement* position_node = node->FirstChildElement("position");
         position_node;
         position_node = position_node->NextSiblingElement("position")) {
      ParseMotorOrPosition(position_node);
    }
    // Unsupported elements are listed in the order from the MuJoCo docs:
    // https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator
    WarnUnsupportedElement(*node, "general");
    WarnUnsupportedElement(*node, "velocity");
    WarnUnsupportedElement(*node, "intvelocity");
    WarnUnsupportedElement(*node, "damper");
    WarnUnsupportedElement(*node, "cylinder");
    WarnUnsupportedElement(*node, "muscle");
    WarnUnsupportedElement(*node, "adhesion");
    WarnUnsupportedElement(*node, "plugin");
  }

  void ParseJoint(XMLElement* node, const RigidBody<double>& parent,
                  const RigidBody<double>& child, const RigidTransformd& X_WC,
                  const RigidTransformd& X_PC,
                  const std::string& child_class = "") {
    std::string name;
    if (!ParseStringAttribute(node, "name", &name)) {
      // Use "parent-body" as the default joint name.
      name = fmt::format("{}-{}", parent.name(), child.name());
    }

    std::string class_name;
    if (!ParseStringAttribute(node, "class", &class_name)) {
      class_name = child_class.empty() ? "main" : child_class;
    }
    ApplyDefaultAttributes("joint", class_name, node);

    Vector3d pos = Vector3d::Zero();
    ParseVectorAttribute(node, "pos", &pos);
    // Drake wants the joint position in the parent frame, but MuJoCo specifies
    // it in the child body frame.
    RigidTransformd X_CJ(pos);
    const RigidTransformd X_PJ = X_PC * X_CJ;

    Vector3d axis = Vector3d::UnitZ();
    ParseVectorAttribute(node, "axis", &axis);
    axis.normalize();
    // Drake wants the axis in the parent frame, but MuJoCo specifies it in the
    // child body frame. But, by definition, these are always the same for
    // revolute(hinge) joint and prismatic(slide) joint because the axis is the
    // constraint that defines the joint.  For ball joint and free joint, the
    // axis attribute is ignored.

    double damping{0.0};
    ParseScalarAttribute(node, "damping", &damping);

    std::string type;
    if (!ParseStringAttribute(node, "type", &type)) {
      type = "hinge";
    }

    Vector2d range(0.0, 0.0);
    bool limited = ParseLimits(node, "range", "limited", &range);

    JointIndex index;
    if (type == "free") {
      if (damping != 0.0) {
        Warning(*node,
                fmt::format(
                    "Damping was specified for the 'free' joint {}, but is not "
                    "supported for free bodies.",
                    name));
      }
      plant_->SetDefaultFloatingBaseBodyPose(child, X_WC);
    } else if (type == "ball") {
      index =
          plant_
              ->AddJoint<BallRpyJoint>(name, parent, X_PJ, child, X_CJ, damping)
              .index();
      if (limited) {
        WarnUnsupportedAttribute(*node, "range");
      }
    } else if (type == "slide") {
      double ref{0.0};
      ParseScalarAttribute(node, "ref", &ref);
      // The current configuration should be treated as position = ref instead
      // of position = 0. This is equivalent to inserting a translating of
      // ref*axis between the joint and the child body, and setting the default
      // position to ref (which we do below).
      X_CJ = X_CJ * RigidTransformd(ref * axis);
      index = plant_
                  ->AddJoint<PrismaticJoint>(
                      name, parent, X_PJ, child, X_CJ, axis,
                      -std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::infinity(), damping)
                  .index();
      if (limited) {
        plant_->get_mutable_joint(index).set_position_limits(
            Vector1d{range[0]}, Vector1d{range[1]});
      }
      plant_->get_mutable_joint(index).set_default_positions(Vector1d{ref});
    } else if (type == "hinge") {
      double ref{0.0};
      if (ParseScalarAttribute(node, "ref", &ref) && angle_ == kDegree) {
        ref *= (M_PI / 180.0);
      }
      // The current configuration should be treated as position = ref instead
      // of position = 0. This is equivalent to rotating the joint frame by
      // -ref and setting the default position to ref (which we do below).
      X_CJ = X_CJ * RigidTransformd(AngleAxisd(ref, axis), Vector3d::Zero());
      index = plant_
                  ->AddJoint<RevoluteJoint>(name, parent, X_PJ, child, X_CJ,
                                            axis, damping)
                  .index();
      if (limited) {
        if (angle_ == kDegree) {
          range *= (M_PI / 180.0);
        }
        plant_->get_mutable_joint(index).set_position_limits(
            Vector1d{range[0]}, Vector1d{range[1]});
      }
      plant_->get_mutable_joint(index).set_default_positions(Vector1d{ref});
    } else {
      Error(*node, "Unknown joint type " + type);
      return;
    }

    // Note: The MuJoCo docs state that the armature is *always* added to all
    // dofs of the inertia matrix: "Armature inertia (or rotor inertia, or
    // reflected inertia) of all degrees of freedom created by this joint.
    // These are constants added to the diagonal of the inertia matrix in
    // generalized coordinates." But that is not what the rotor inertia of a
    // motor actually does. In Drake we only add the rotor inertia through the
    // joint actuator. This is a modeling difference.
    double armature{0.0};
    if (index.is_valid() && ParseScalarAttribute(node, "armature", &armature)) {
      // Stash armature value to be used when parsing actuators.
      armature_.emplace(index, armature);
    }

    // Unsupported attributes are listed in the order from the MuJoCo docs:
    // https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint
    // The ignored attributes are specific to the MuJoCo solver.
    WarnUnsupportedAttribute(*node, "group");
    WarnUnsupportedAttribute(*node, "springdamper");
    LogIgnoredAttribute(*node, "solreflimit");
    LogIgnoredAttribute(*node, "solimplimit");
    LogIgnoredAttribute(*node, "solreffriction");
    LogIgnoredAttribute(*node, "solimpfriction");
    WarnUnsupportedAttribute(*node, "stiffness");
    WarnUnsupportedAttribute(*node, "actuatorfrcrange");
    WarnUnsupportedAttribute(*node, "actuatorfrclimited");
    WarnUnsupportedAttribute(*node, "actuatorgravcomp");
    LogIgnoredAttribute(*node, "margin");
    WarnUnsupportedAttribute(*node, "springref");
    WarnUnsupportedAttribute(*node, "frictionloss");
    WarnUnsupportedAttribute(*node, "user");
  }

  // Computes a shape's volume, centroid, and unit inertia. This calculator is
  // used in a context where the calculation of mass needs to be deferred (e.g.,
  // density is not yet determined or mass has been explicitly specified). The
  // caller is responsible for defining the final value for mass, e.g., using a
  // parser-specified mass or density (or a fallback default density).
  class InertiaCalculator final : public geometry::ShapeReifier {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InertiaCalculator);
    // The reifier aliases the pre-computed mesh spatial inertias. When looking
    // up mesh inertias, it uses the mujoco geometry _name_ and not the
    // mesh filename.
    InertiaCalculator(
        const DiagnosticPolicy& policy, std::string name,
        std::map<std::string, SpatialInertia<double>>* mesh_inertia)
        : policy_(policy), name_(std::move(name)), mesh_inertia_(mesh_inertia) {
      DRAKE_DEMAND(mesh_inertia != nullptr);
    }

    // Returns the tuple [volume, p_GoGcm_G, G_GGo_G], where for a geometry G,
    // volume is G's volume, p_GoGcm_G is the position from G's origin point Go
    // to its centroid Gcm expressed in the G frame, and G_GGo_G is G's unit
    // inertia about Go expressed in the G frame.
    // Note: if G has uniform density, Gcm is G's center of mass.
    std::tuple<double, Vector3d, UnitInertia<double>> Calc(
        const geometry::Shape& shape) {
      shape.Reify(this);
      // For unit density (1 kg/m³), the value of mass is equal to the value of
      // volume.
      const double& volume = M_GGo_G_unitDensity.get_mass();
      const Vector3<double>& p_GoGcm_G = M_GGo_G_unitDensity.get_com();
      const UnitInertia<double>& G_GGo_G =
          M_GGo_G_unitDensity.get_unit_inertia();
      return {volume, p_GoGcm_G, G_GGo_G};
    }

    bool used_convex_hull_fallback() const {
      return used_convex_hull_fallback_;
    }

    using geometry::ShapeReifier::ImplementGeometry;

    void ImplementGeometry(const geometry::Mesh& mesh, void*) final {
      if (mesh_inertia_->contains(name_)) {
        M_GGo_G_unitDensity = mesh_inertia_->at(name_);
      } else {
        CalcSpatialInertiaResult result = CalcSpatialInertiaWithFallback(
            mesh, /* density= */ 1.0,
            /* warn_on_convex= */ [this](const std::string& message) {
              used_convex_hull_fallback_ = true;
              policy_.Warning(message);
            });

        if (std::holds_alternative<std::string>(result)) {
          policy_.Error(fmt::format(
              "Failed to compute spatial inertia even for the convex hull of "
              "{}.\n{}",
              mesh.source().path().string(), std::get<std::string>(result)));
        } else {
          M_GGo_G_unitDensity = std::get<SpatialInertia<double>>(result);
        }
      }
      mesh_inertia_->insert_or_assign(name_, M_GGo_G_unitDensity);
    }

    void ImplementGeometry(const geometry::HalfSpace&, void*) final {
      // Do nothing; leave M_GGo_G_unitDensity default initialized.
    }

    void DefaultImplementGeometry(const geometry::Shape& shape) final {
      M_GGo_G_unitDensity = CalcSpatialInertia(shape, 1.0 /* density */);
    }

   private:
    const DiagnosticPolicy& policy_;
    std::string name_;
    std::map<std::string, SpatialInertia<double>>* mesh_inertia_{nullptr};
    bool used_convex_hull_fallback_{false};
    // Note: The spatial inertia below uses unit density so that the shape's
    // volume value is equal to its mass value. To be clear, unit density is a
    // mathematical trick to use CalcSpatialInertia() to report volume and not
    // a reasonable *physical* default value; 1 kg/m³ is approximately air's
    // density.
    SpatialInertia<double> M_GGo_G_unitDensity{SpatialInertia<double>::NaN()};
  };

  SpatialInertia<double> ParseInertial(XMLElement* node) {
    // We use F to denote the "inertial frame" in the MujoCo documentation.  B
    // is the body frame.
    RigidTransformd X_BF = ParseTransform(node);
    double mass;
    if (!ParseScalarAttribute(node, "mass", &mass)) {
      Error(*node, "The inertial tag must include the mass attribute.");
      return SpatialInertia<double>::NaN();
    }

    // We interpret the MuJoCo XML documentation as saying that if a
    // diaginertia is provided, it is I_BFo_F.  If a full inertia is provided,
    // then it must be I_BFo_B (since the inertia is always diagonal in F).
    RotationalInertia<double> I_BFo_B;

    Vector3d diag;
    if (ParseVectorAttribute(node, "diaginertia", &diag)) {
      I_BFo_B = RotationalInertia<double>(diag[0], diag[1], diag[2])
                    .ReExpress(X_BF.rotation());
    } else {
      // fullinertia is required if diaginertia is not specified.
      Vector6d full;
      if (ParseVectorAttribute(node, "fullinertia", &full)) {
        I_BFo_B = RotationalInertia<double>(full[0], full[1], full[2], full[3],
                                            full[4], full[5]);
      } else {
        Error(*node,
              "The inertial tag must include either the diaginertia or "
              "fullinertia attribute.");
        return SpatialInertia<double>::NaN();
      }
    }

    SpatialInertia<double> M_BBo_B =
        SpatialInertia<double>::MakeFromCentralInertia(mass, X_BF.translation(),
                                                       I_BFo_B);
    return M_BBo_B;
  }

  struct MujocoGeometry {
    RigidTransformd X_BG{};
    std::string name{};
    std::unique_ptr<geometry::Shape> shape{};
    Vector4d rgba{.5, .5, .5, 1};
    CoulombFriction<double> friction{1.0, 1.0};
    SpatialInertia<double> M_GBo_B{SpatialInertia<double>::NaN()};
    bool register_collision{true};
    bool register_visual{true};
  };

  MujocoGeometry ParseGeometry(XMLElement* node, int num_geom,
                               bool compute_inertia,
                               const std::string& child_class = "") {
    MujocoGeometry geom;

    std::string class_name;
    if (!ParseStringAttribute(node, "class", &class_name)) {
      class_name = child_class.empty() ? "main" : child_class;
    }
    // TODO(russt): Add a test case covering childclass/default nesting once
    // the body element is supported.
    ApplyDefaultAttributes("geom", class_name, node);

    // Per the MuJoCo documentation, the name is not part of the defaults. This
    // is consistent with Drake requiring that geometry names are unique.
    if (!ParseStringAttribute(node, "name", &geom.name)) {
      // Use "geom#" as the default body name.
      geom.name = fmt::format("geom{}", num_geom);
    }

    geom.X_BG = ParseTransform(node);

    std::string type;
    if (!ParseStringAttribute(node, "type", &type)) {
      type = "sphere";
    }

    // Note: The documentation says e.g. for sphere "only one size parameter is
    // used", but it seems that in practice often more than one are specified.
    // Demanding the correct number of size parameters is too restrictive.
    std::vector<double> size;
    {
      std::string size_attr;
      ParseStringAttribute(node, "size", &size_attr);
      size = ConvertToVector<double>(size_attr);
    }

    Vector6d fromto;
    bool has_fromto = ParseVectorAttribute(node, "fromto", &fromto);

    if (has_fromto) {
      // Set the pose to the midpoint of the from-to vector, with the
      // orientation set according to the "zaxis" specification in
      // ParseTransform.
      const Vector3d from = fromto.head<3>();
      const Vector3d to = fromto.tail<3>();
      geom.X_BG = RigidTransformd(Eigen::Quaternion<double>::FromTwoVectors(
                                      Vector3d{0, 0, 1}, to - from),
                                  (from + to) / 2);
    } else {
      geom.X_BG = ParseTransform(node);
    }

    std::string mesh;
    bool has_mesh_attribute = ParseStringAttribute(node, "mesh", &mesh);
    if (has_mesh_attribute && type != "mesh") {
      if (type == "sphere" || type == "capsule" || type == "cylinder" ||
          type == "ellipsoid" || type == "box") {
        Error(*node,
              fmt::format(
                  "geom {} specified type '{}' but also has a mesh attribute. "
                  "The intended behavior is to compute the size of the shape "
                  "from the mesh, but this is not supported yet (#22372).",
                  geom.name, type));
        return geom;
      } else {
        Error(*node, fmt::format("geom {} specified a 'mesh', but this is not "
                                 "allowed for type '{}'.",
                                 geom.name, type));
        return geom;
      }
    }
    if (type == "plane") {
      // We interpret the MuJoCo infinite plane as a half-space.
      geom.shape = std::make_unique<geometry::HalfSpace>();
      compute_inertia = false;
    } else if (type == "sphere") {
      if (size.size() < 1) {
        // Allow zero-radius spheres (the MJCF default size is 0 0 0).
        geom.shape = std::make_unique<geometry::Sphere>(0.0);
        compute_inertia = false;
      } else {
        geom.shape = std::make_unique<geometry::Sphere>(size[0]);
      }
    } else if (type == "capsule") {
      if (has_fromto) {
        if (size.size() < 1) {
          Error(*node,
                "The size attribute for capsule geom using fromto must have at "
                "least one element.");
          return geom;
        }
        double length = (fromto.head<3>() - fromto.tail<3>()).norm();
        geom.shape = std::make_unique<geometry::Capsule>(size[0], length);

      } else {
        if (size.size() < 2) {
          Error(*node,
                "The size attribute for capsule geom must have at least two "
                "elements.");
          return geom;
        }
        geom.shape = std::make_unique<geometry::Capsule>(size[0], 2 * size[1]);
      }
    } else if (type == "ellipsoid") {
      if (has_fromto) {
        // The specification is not properly documented in the XML schema.
        Warning(*node, fmt::format("The fromto tag for ellipsoid is currently "
                                   "unsupported; geom {} will be ignored.",
                                   geom.name));
        return geom;
      } else {
        if (size.size() < 3) {
          Error(
              *node,
              "The size attribute for ellipsoid geom must have at least three "
              "elements.");
          return geom;
        }
        geom.shape =
            std::make_unique<geometry::Ellipsoid>(size[0], size[1], size[2]);
      }
    } else if (type == "cylinder") {
      if (has_fromto) {
        if (size.size() < 1) {
          Error(*node,
                "The size attribute for cylinder geom using fromto must have "
                "at least one element.");
          return geom;
        }
        double length = (fromto.head<3>() - fromto.tail<3>()).norm();
        geom.shape = std::make_unique<geometry::Cylinder>(size[0], length);
      } else {
        if (size.size() < 2) {
          Error(*node,
                "The size attribute for cylinder geom must have at least two "
                "elements.");
          return geom;
        }
        geom.shape = std::make_unique<geometry::Cylinder>(size[0], 2 * size[1]);
      }
    } else if (type == "box") {
      if (has_fromto) {
        // The specification is not properly documented in the XML schema.
        Warning(*node, fmt::format("The fromto tag for box is currently "
                                   "unsupported; geom {} will be "
                                   "ignored.",
                                   geom.name));
        return geom;
      } else {
        if (size.size() < 3) {
          Error(*node,
                "The size attribute for box geom must have at least three "
                "elements.");
          return geom;
        }
        geom.shape = std::make_unique<geometry::Box>(
            size[0] * 2.0, size[1] * 2.0, size[2] * 2.0);
      }
    } else if (type == "mesh") {
      if (!has_mesh_attribute) {
        Error(*node, fmt::format("geom {} specified type 'mesh', but did not "
                                 "set the mesh attribute",
                                 geom.name));
        return geom;
      }
      if (mesh_.contains(mesh)) {
        geom.shape = mesh_.at(mesh)->Clone();
      } else {
        Warning(
            *node,
            fmt::format("geom {} specified unknown mesh {} and will be ignored",
                        geom.name, mesh));
        return geom;
      }
    } else if (type == "hfield") {
      Warning(*node, fmt::format("The geom type '{}' is currently unsupported "
                                 "and will be ignored.",
                                 type));
      return geom;
    } else {
      Error(*node, fmt::format("Unrecognized geom type {}", type));
      return geom;
    }

    int contype{1};
    int conaffinity{1};
    int condim{3};
    ParseScalarAttribute(node, "contype", &contype);
    ParseScalarAttribute(node, "conaffinity", &conaffinity);
    ParseScalarAttribute(node, "condim", &condim);
    if (contype == 0 && conaffinity == 0) {
      // This is a common mechanism used by MJCF authors to specify visual-only
      // geometry.
      geom.register_collision = false;
    } else if (contype != 1 || conaffinity != 1) {
      Warning(
          *node,
          fmt::format(
              "geom {} specified contype={} and conaffinity={}; but collision "
              "filter groups are not yet implemented for the mujoco parser.",
              geom.name, contype, conaffinity));
    }
    if (condim != 3) {
      // TODO(russt): Can we support condim=1 inefficiently by setting friction
      // coefficients to zero? This may be insufficient given the parallel
      // resistor friction coefficient logic. condim=4 or 6 are likely out of
      // scope for now.
      Warning(*node, fmt::format("geom {} specified condim={}, which is not "
                                 "supported. condim=3 will be used instead.",
                                 geom.name, condim));
    }

    if (geom.register_collision && type == "sphere" && size.size() < 1) {
      Warning(*node,
              fmt::format(
                  "Using zero-radius spheres (MuJoCo's default geometry) for "
                  "collision geometry may not be supported by all features in "
                  "Drake. Consider specifying a non-zero size for geom {}.",
                  geom.name));
    }

    int group{0};
    ParseScalarAttribute(node, "group", &group);
    if (group > 2) {
      // By default, the MuJoCo visualizer does not render geom with group > 2.
      // Setting the group > 2 is a common mechanism that MuJoCo uses to
      // register collision-only geometry.
      geom.register_visual = false;
      // TODO(russt): Consider adding a <drake::enable_visual_for_group> tag so
      // that mjcf authors can configure this behavior.
    }

    std::string material;
    if (ParseStringAttribute(node, "material", &material)) {
      if (material_.contains(material)) {
        XMLElement* material_node = material_.at(material);
        // Note: there are many material attributes that we do not support yet,
        // nor perhaps ever. Consider warning about them here (currently, it
        // seems like too much noise).
        ParseVectorAttribute(material_node, "rgba", &geom.rgba);
      } else {
        Warning(*node,
                fmt::format("geom {} specified an unrecognized material {}",
                            geom.name, material));
      }
    }
    // rgba takes precedence over materials, so must be parsed after.
    ParseVectorAttribute(node, "rgba", &geom.rgba);

    // Note: The documentation suggests that at least 3 friction parameters
    // should be specified.  But humanoid_CMU.xml in the DeepMind suite
    // specifies only one, and in fact we only need one.
    std::vector<double> friction;
    {
      std::string friction_attr;
      ParseStringAttribute(node, "friction", &friction_attr);
      friction = ConvertToVector<double>(friction_attr);
    }
    if (!friction.empty()) {
      // MuJoCo's friction specification is [sliding, torsional, rolling].  We
      // set the static and dynamic friction to `sliding`, and do not support
      // the other parameters.
      geom.friction = CoulombFriction(friction[0], friction[0]);
      if ((friction.size() > 1 && friction[1] != 0.0) ||
          (friction.size() > 2 && friction[2] != 0.0)) {
        Warning(
            *node,
            fmt::format(
                "The torsional and rolling friction specified in the friction "
                "attribute of {} are unsupported and will be ignored.",
                node->Name()));
      }
    }

    // Unsupported attributes are listed in the order from the MuJoCo docs:
    // https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-geom
    // The ignored attributes are specific to the MuJoCo solver.
    LogIgnoredAttribute(*node, "priority");
    WarnUnsupportedAttribute(*node, "shellinertia");
    LogIgnoredAttribute(*node, "solmix");
    LogIgnoredAttribute(*node, "solref");
    LogIgnoredAttribute(*node, "solimp");
    LogIgnoredAttribute(*node, "margin");
    LogIgnoredAttribute(*node, "gap");
    // The hfield attribute is covered by the type=="hfield" warning.
    WarnUnsupportedAttribute(*node, "fitscale");
    WarnUnsupportedAttribute(*node, "fluidshape");
    WarnUnsupportedAttribute(*node, "fluidcoef");
    WarnUnsupportedAttribute(*node, "user");

    if (compute_inertia) {
      auto policy = diagnostic_.MakePolicyForNode(node);
      InertiaCalculator calculator(policy, mesh, &mesh_inertia_);
      const auto [volume, p_GoGcm_G, G_GGo_G] = calculator.Calc(*geom.shape);
      if (calculator.used_convex_hull_fallback()) {
        // When the Mujoco parser falls back to using a convex hull, it prints
        // a warning. We ape that behavior to provide a similar experience.
        //
        // N.B. Mujoco falls back only if the mesh reported a negative volume.
        // SpatialInertia::IsPhysicallyValid() will also fail in that case. But
        // fails in other cases (e.g., positive mass but negative inertia) where
        // Drake will fallback and Mujoco won't. Such a mesh would be rare.
        // Generally, we expect Drake and Mujoco to agree on real world meshes.
        Warning(
            *node,
            fmt::format("CalcSpatialInertia() failed to compute a physically "
                        "valid inertia for mesh {} (probably the mesh is not "
                        "watertight). The spatial inertia was computed using "
                        "its convex hull as a fallback.",
                        mesh));
      }
      double mass{};
      if (!ParseScalarAttribute(node, "mass", &mass)) {
        double density{1000}; /* fallback default ≈ water density */
        ParseScalarAttribute(node, "density", &density);
        mass = volume * density;
      }
      SpatialInertia<double> M_GGo_G(mass, p_GoGcm_G, G_GGo_G,
                                     /*skip_validity_check=*/true);
      std::optional<std::string> invalidity_report =
          M_GGo_G.CreateInvalidityReport();
      if (invalidity_report.has_value()) {
        Error(*node, fmt::format("geom {} {}", geom.name, *invalidity_report));
        return geom;
      }

      // Shift spatial inertia from Go to Bo and express it in the B frame.
      const math::RotationMatrix<double>& R_BG = geom.X_BG.rotation();
      const Vector3<double>& p_BoGo_B = geom.X_BG.translation();
      geom.M_GBo_B = M_GGo_G.ReExpress(R_BG).Shift(-p_BoGo_B);
    }
    return geom;
  }

  void ParseCamera(XMLElement* node, const RigidBody<double>& parent,
                   const std::string& child_class = "") {
    if (builder_ == nullptr || scene_graph_ == nullptr) {
      Warning(*node,
              "camera element ignored; to register cameras, you must pass a "
              "DiagramBuilder to the Parser and the plant must be registered "
              "with a scene graph.");
      return;
    }

    std::string class_name;
    if (!ParseStringAttribute(node, "class", &class_name)) {
      class_name = child_class.empty() ? "main" : child_class;
    }
    ApplyDefaultAttributes("camera", class_name, node);

    systems::sensors::CameraConfig config;

    std::string name;
    if (!ParseStringAttribute(node, "name", &name)) {
      name = fmt::format("camera{}", num_cameras_);
    }
    // We add the model instance name as a prefix to the camera name.
    config.name = fmt::format(
        "{}/{}", plant_->GetModelInstanceName(model_instance_), name);

    std::string mode;
    if (ParseStringAttribute(node, "mode", &mode)) {
      if (mode != "fixed") {
        Warning(*node,
                fmt::format("camera {} requested mode '{}', which is not "
                            "supported yet. A fixed mode will be used instead.",
                            config.name, mode));
      }
    }

    std::string orthographic;
    if (ParseStringAttribute(node, "orthographic", &orthographic)) {
      if (orthographic == "true") {
        Warning(
            *node,
            fmt::format(
                "camera {} requested orthographic projection, which is not "
                "supported yet. A perspective projection will be used instead.",
                config.name));
      }
    }

    config.X_PB = schema::Transform(ParseTransform(node));
    config.X_PB.base_frame = parent.body_frame().scoped_name().get_full();

    double fovy{45};
    ParseScalarAttribute(node, "fovy", &fovy);
    // MuJoCo's fovy is always in degrees; it does not follow the
    // compiler/angle setting.
    config.focal = systems::sensors::CameraConfig::FovDegrees{.y = fovy};

    // MuJoCo's specified a default resolution of (1,1); but Drake doesn't
    // support this (and it's a silly default). We'll use Drake's default
    // resolution unless a resolution is specified explicitly.
    Vector2d resolution;
    if (ParseVectorAttribute(node, "resolution", &resolution)) {
      config.width = resolution[0];
      config.height = resolution[1];
    }

    // TODO(russt): Support drake-specific elements/attributes to e.g. set the
    // renderer name and other Drake camera configurable parameters.

    // Note: we're opting-out of LCM by passing lcm_buses = nullptr and a
    // throw-away DrakeLcm instance.
    lcm::DrakeLcm lcm(systems::lcm::LcmBuses::kLcmUrlMemqNull);
    ApplyCameraConfig(config, builder_, nullptr, plant_, scene_graph_, &lcm);
    ++num_cameras_;

    // Unsupported elements are listed in the order from the MuJoCo docs:
    // https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-camera
    WarnUnsupportedAttribute(*node, "target");
    WarnUnsupportedAttribute(*node, "focal");
    WarnUnsupportedAttribute(*node, "focalpixel");
    WarnUnsupportedAttribute(*node, "principal");
    WarnUnsupportedAttribute(*node, "principalpixel");
    WarnUnsupportedAttribute(*node, "sensorsize");
    WarnUnsupportedAttribute(*node, "ipd");
    WarnUnsupportedAttribute(*node, "user");
  }

  void ParseBody(XMLElement* node, const RigidBody<double>& parent,
                 const RigidTransformd& X_WP,
                 const std::string& parent_class = "") {
    std::string body_name;
    if (!ParseStringAttribute(node, "name", &body_name)) {
      // Use "body#" as the default body name.
      body_name = fmt::format("body{}", plant_->num_bodies());
    }

    std::string child_class;
    if (!ParseStringAttribute(node, "childclass", &child_class)) {
      child_class = parent_class;
    }

    bool compute_inertia{};
    SpatialInertia<double> M_BBo_B(0, {0, 0, 0}, {0, 0, 0});
    XMLElement* inertial_node = node->FirstChildElement("inertial");
    if (inertial_node && (inertia_from_geom_ != kTrue)) {
      // Then we have a node and inertial_from_geom is "auto" or "false".
      M_BBo_B = ParseInertial(inertial_node);
      compute_inertia = false;
    } else if (!inertial_node && (inertia_from_geom_ == kFalse)) {
      // We don't have a node and inertial_from_geom is "false".
      // https://mujoco.readthedocs.io/en/latest/XMLreference.html#compiler
      // says we should emit an error.
      Error(*node,
            fmt::format("{} has no inertial tag and inertiafromgeom=false. You "
                        "must specify an inertia.",
                        body_name));
      return;
    } else {
      compute_inertia = true;
    }

    // Note: Because AddRigidBody returns a const RigidBody, I must know the
    // (default) SpatialInertia at construction time.  So I'm forced to parse
    // the geometry into an intermediate representation (to possibly compute the
    // inertia), then add the body, then add the geometry.

    // Parses geom elements.
    std::vector<MujocoGeometry> geometries;
    for (XMLElement* link_node = node->FirstChildElement("geom"); link_node;
         link_node = link_node->NextSiblingElement("geom")) {
      auto geom = ParseGeometry(link_node, geometries.size(), compute_inertia,
                                child_class);
      if (!geom.shape) continue;
      if (compute_inertia && geom.M_GBo_B.get_mass() > 0) {
        M_BBo_B += geom.M_GBo_B;
      }
      geometries.push_back(std::move(geom));
    }

    // Add a rigid body to model each link.
    const RigidBody<double>& body =
        plant_->AddRigidBody(body_name, model_instance_, M_BBo_B);

    if (plant_->geometry_source_is_registered()) {
      for (auto& geom : geometries) {
        if (geom.register_visual) {
          plant_->RegisterVisualGeometry(body, geom.X_BG, *geom.shape,
                                         geom.name, geom.rgba);
        }
        if (geom.register_collision) {
          plant_->RegisterCollisionGeometry(body, geom.X_BG, *geom.shape,
                                            geom.name, geom.friction);
        }
      }
    }

    const RigidTransformd X_PB = ParseTransform(node);
    const RigidTransformd X_WB = X_WP * X_PB;

    XMLElement* joint_node = node->FirstChildElement("joint");
    if (joint_node) {
      // We apply joint sequentially with a dummy body inserted.  Each joint is
      // in the coordinates of the parent body, until the last, which
      // transforms to this body's coordinates.
      const RigidBody<double>* last_body = &parent;
      int dummy_bodies = 0;
      for (; joint_node; joint_node = joint_node->NextSiblingElement("joint")) {
        if (joint_node->NextSiblingElement("joint")) {
          const RigidBody<double>& dummy_body = plant_->AddRigidBody(
              fmt::format("{}{}", body_name, dummy_bodies++), model_instance_,
              SpatialInertia<double>::Zero());
          ParseJoint(joint_node, *last_body, dummy_body, X_WP,
                     RigidTransformd(), child_class);
          last_body = &dummy_body;
        } else {
          ParseJoint(joint_node, *last_body, body, X_WB, X_PB, child_class);
        }

        std::string type;
        ParseStringAttribute(joint_node, "type", &type);
        if (type == "free") {
          if (dummy_bodies > 0) {
            Error(*node, fmt::format(
                             "No other joints can be defined in the body {} if "
                             "a free joint is defined.",
                             body_name));
            return;
          }
          break;  // No other joints are allowed if the joint is "free".
        }
      }
    } else {  // no "joint" element.
      if (XMLElement* freejoint_node = node->FirstChildElement("freejoint")) {
        WarnUnsupportedElement(*freejoint_node, "name");
        WarnUnsupportedElement(*freejoint_node, "group");
        plant_->SetDefaultFloatingBaseBodyPose(body, X_WB);
      } else {
        plant_->WeldFrames(parent.body_frame(), body.body_frame(), X_PB);
      }
    }

    for (XMLElement* camera_node = node->FirstChildElement("camera");
         camera_node; camera_node = camera_node->NextSiblingElement("camera")) {
      ParseCamera(camera_node, body, child_class);
    }

    // Unsupported attributes are listed in the order from the MuJoCo docs:
    // https://mujoco.readthedocs.io/en/stable/XMLreference.html#body
    WarnUnsupportedAttribute(*node, "mocap");
    WarnUnsupportedAttribute(*node, "gravcomp");
    WarnUnsupportedAttribute(*node, "user");
    WarnUnsupportedElement(*node, "site");
    WarnUnsupportedElement(*node, "light");
    WarnUnsupportedElement(*node, "composite");
    WarnUnsupportedElement(*node, "flexcomp");
    WarnUnsupportedElement(*node, "plugin");
    WarnUnsupportedElement(*node, "attach");
    WarnUnsupportedElement(*node, "frame");

    // Parses child body elements.
    for (XMLElement* link_node = node->FirstChildElement("body"); link_node;
         link_node = link_node->NextSiblingElement("body")) {
      ParseBody(link_node, body, X_WB, child_class);
    }
  }

  void ParseWorldBody(XMLElement* node) {
    if (plant_->geometry_source_is_registered()) {
      int num_geometries = 0;
      for (XMLElement* link_node = node->FirstChildElement("geom"); link_node;
           link_node = link_node->NextSiblingElement("geom")) {
        auto geom = ParseGeometry(link_node, num_geometries, false);
        if (!geom.shape) continue;
        if (geom.register_visual) {
          plant_->RegisterVisualGeometry(plant_->world_body(), geom.X_BG,
                                         *geom.shape, geom.name, geom.rgba);
        }
        if (geom.register_collision) {
          plant_->RegisterCollisionGeometry(plant_->world_body(), geom.X_BG,
                                            *geom.shape, geom.name,
                                            geom.friction);
        }
        ++num_geometries;
      }
    }

    for (XMLElement* camera_node = node->FirstChildElement("camera");
         camera_node; camera_node = camera_node->NextSiblingElement("camera")) {
      ParseCamera(camera_node, plant_->world_body());
    }

    // Unsupported attributes are listed in the order from the MuJoCo docs:
    // https://mujoco.readthedocs.io/en/stable/XMLreference.html#body
    WarnUnsupportedAttribute(*node, "mocap");
    WarnUnsupportedAttribute(*node, "gravcomp");
    WarnUnsupportedAttribute(*node, "user");
    WarnUnsupportedElement(*node, "site");
    WarnUnsupportedElement(*node, "light");
    WarnUnsupportedElement(*node, "composite");
    WarnUnsupportedElement(*node, "flexcomp");
    WarnUnsupportedElement(*node, "plugin");
    WarnUnsupportedElement(*node, "attach");
    WarnUnsupportedElement(*node, "frame");

    // Parses child body elements.
    for (XMLElement* link_node = node->FirstChildElement("body"); link_node;
         link_node = link_node->NextSiblingElement("body")) {
      ParseBody(link_node, plant_->world_body(), RigidTransformd());
    }
  }

  // Parse sub-elements of `<default>`, for a particular `element name`,
  // updating `default_map`. Implements the inheritance mechanism for
  // defaults; see
  // https://mujoco.readthedocs.io/en/latest/modeling.html#cdefault
  void ParseClassDefaults(XMLElement* node, const std::string& class_name,
                          const std::string& parent_default,
                          const std::string& element_name) {
    std::pair<std::string, std::string> key{element_name, class_name};
    const char* elt_name = element_name.c_str();
    for (XMLElement* e = node->FirstChildElement(elt_name); e;
         e = e->NextSiblingElement(elt_name)) {
      defaults_[key] = e;
      if (!parent_default.empty()) {
        ApplyDefaultAttributes(element_name, parent_default, e);
      }
    }
  }

  void ParseDefault(XMLElement* node, const std::string& parent_default = "") {
    std::string class_name;
    if (!ParseStringAttribute(node, "class", &class_name)) {
      if (parent_default.empty()) {
        class_name = "main";
      } else {
        Error(*node,
              "The `class` attribute is required for all `default` elements "
              "except at the top-level");
        return;
      }
    }

    // This sugar forwards common local arguments to ParseClassDefaults().
    auto parse_class_defaults = [&](const std::string& element_name) {
      ParseClassDefaults(node, class_name, parent_default, element_name);
    };

    // https://mujoco.readthedocs.io/en/stable/XMLreference.html#default-r
    parse_class_defaults("geom");
    parse_class_defaults("joint");
    parse_class_defaults("mesh");
    parse_class_defaults("equality");
    parse_class_defaults("material");
    parse_class_defaults("site");
    parse_class_defaults("camera");
    parse_class_defaults("light");
    parse_class_defaults("pair");
    parse_class_defaults("tendon");
    parse_class_defaults("general");
    parse_class_defaults("motor");
    parse_class_defaults("position");
    parse_class_defaults("velocity");
    parse_class_defaults("cylinder");
    parse_class_defaults("muscle");

    // Parse child defaults.
    for (XMLElement* default_node = node->FirstChildElement("default");
         default_node;
         default_node = default_node->NextSiblingElement("default")) {
      ParseDefault(default_node, class_name);
    }
  }

  void ParseAsset(XMLElement* node) {
    for (XMLElement* material_node = node->FirstChildElement("material");
         material_node;
         material_node = material_node->NextSiblingElement("material")) {
      std::string name;
      if (!ParseStringAttribute(material_node, "name", &name)) {
        Error(*node, "ERROR: Material elements must have a name attribute.");
        return;
      }
      material_[name] = material_node;
    }

    for (XMLElement* mesh_node = node->FirstChildElement("mesh"); mesh_node;
         mesh_node = mesh_node->NextSiblingElement("mesh")) {
      std::string class_name;
      if (!ParseStringAttribute(mesh_node, "class", &class_name)) {
        class_name = "main";
      }
      ApplyDefaultAttributes("mesh", class_name, mesh_node);

      // Unsupported attributes are listed in the order from the MuJoCo docs:
      // https://mujoco.readthedocs.io/en/stable/XMLreference.html#mesh
      WarnUnsupportedAttribute(*mesh_node, "smoothnormal");
      WarnUnsupportedAttribute(*mesh_node, "maxhullvert");
      WarnUnsupportedAttribute(*mesh_node, "vertex");
      // Note: "normal", "face", and "texcoord" are not supported either, but
      // that lack of support is implied by us not supporting "vertex".
      WarnUnsupportedAttribute(*mesh_node, "refpos");
      WarnUnsupportedAttribute(*mesh_node, "refquat");
      WarnUnsupportedElement(*mesh_node, "plugin");

      std::string file;
      if (ParseStringAttribute(mesh_node, "file", &file)) {
        std::string name;
        if (!ParseStringAttribute(mesh_node, "name", &name)) {
          // Per the mujoco docs, if the "name" attribute is omitted then the
          // mesh name equals the file name without the path and extension.
          name = std::filesystem::path(file).stem();
        }

        Vector3d scale{1, 1, 1};
        ParseVectorAttribute(mesh_node, "scale", &scale);

        std::filesystem::path filename(file);

        /* Adapted from the mujoco docs: The full path to a file is determined
        as follows. If the strippath compiler option is "true", all path
        information from the file name is removed. The following checks are
        then applied in order: (1) if the file name contains an absolute path,
        it is used without further changes; (2) if the compiler meshdir
        attribute is set and contains an absolute path, the full path is the
        meshdir appended with the file name; (3) the full path is the
        path to the main MJCF model file, appended with the value of meshdir if
        set, appended with the file name. */

        // TODO(russt): Support strippath.
        if (!filename.is_absolute()) {
          if (meshdir_) {
            if (meshdir_->is_absolute()) {
              filename = *meshdir_ / filename;
            } else {
              filename = main_mjcf_path_ / *meshdir_ / filename;
            }
          } else {
            filename = main_mjcf_path_ / filename;
          }
        }
        filename = std::filesystem::weakly_canonical(filename);

        if (std::filesystem::exists(filename)) {
          std::string extension = filename.extension();
          std::transform(extension.begin(), extension.end(), extension.begin(),
                         ::tolower);
          // TODO(russt): Support .vtk files.
          if (extension == ".obj") {
            mesh_[name] = std::make_unique<geometry::Mesh>(filename, scale);
          } else {
            Error(
                *node,
                fmt::format(
                    "Drake's MuJoCo parser currently only supports mesh files "
                    "in .obj format, but the meshfile \"{}\" was requested. "
                    "See https://drake.mit.edu/troubleshooting.html for "
                    "additional resources.",
                    filename.string()));
            continue;
          }
        } else {
          Error(*node, fmt::format("The mesh asset \"{}\" could not be found.",
                                   filename.string()));
          continue;
        }
      } else {
        std::string name;
        ParseStringAttribute(mesh_node, "name", &name);
        Warning(*node, fmt::format("The mesh asset named {} did not specify a "
                                   "'file' attribute and so will be ignored.",
                                   name));
      }
    }

    if (node->FirstChildElement("texture") != nullptr) {
      Warning(*node,
              "The texture element is not supported, see "
              "https://drake.mit.edu/troubleshooting.html for "
              "additional resources.");
    }

    // Unsupported attributes are listed in the order from the MuJoCo docs:
    // https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset
    WarnUnsupportedElement(*node, "hfield");
    WarnUnsupportedElement(*node, "skin");
  }

  void ParseOption(XMLElement* node) {
    Vector3d gravity;
    if (ParseVectorAttribute(node, "gravity", &gravity)) {
      // Note: This changes gravity for the entire plant (including models that
      // existed before this parser).
      plant_->mutable_gravity_field().set_gravity_vector(gravity);
    }

    // Unsupported attributes are listed in the order from the MuJoCo docs:
    // https://mujoco.readthedocs.io/en/stable/XMLreference.html#option
    // Some of these attributes are silently ignored since they are specific to
    // the mujoco solvers, and don't have a direct equivalent in Drake.
    WarnUnsupportedAttribute(*node, "timestep");
    LogIgnoredAttribute(*node, "apirate");
    LogIgnoredAttribute(*node, "impratio");
    WarnUnsupportedAttribute(*node, "wind");
    WarnUnsupportedAttribute(*node, "magnetic");
    WarnUnsupportedAttribute(*node, "density");
    WarnUnsupportedAttribute(*node, "viscosity");
    LogIgnoredAttribute(*node, "o_margin");
    LogIgnoredAttribute(*node, "o_solref");
    LogIgnoredAttribute(*node, "o_solimp");
    LogIgnoredAttribute(*node, "o_friction");
    WarnUnsupportedAttribute(*node, "integrator");
    WarnUnsupportedAttribute(*node, "cone");
    LogIgnoredAttribute(*node, "jacobian");
    LogIgnoredAttribute(*node, "solver");
    LogIgnoredAttribute(*node, "iterations");
    LogIgnoredAttribute(*node, "tolerance");
    LogIgnoredAttribute(*node, "ls_iterations");
    LogIgnoredAttribute(*node, "ls_tolerance");
    LogIgnoredAttribute(*node, "noslip_iterations");
    LogIgnoredAttribute(*node, "noslip_tolerance");
    LogIgnoredAttribute(*node, "ccd_iterations");
    LogIgnoredAttribute(*node, "ccd_tolerance");
    LogIgnoredAttribute(*node, "sdf_iterations");
    LogIgnoredAttribute(*node, "sdf_tolerance");
    LogIgnoredAttribute(*node, "sdf_initpoints");
    WarnUnsupportedAttribute(*node, "actuatorgroupdisable");
    WarnUnsupportedElement(*node, "flag");
  }

  void ParseCompiler(XMLElement* node) {
    autolimits_ = node->BoolAttribute("autolimits", true);

    std::string coordinate;
    if (ParseStringAttribute(node, "coordinate", &coordinate)) {
      if (coordinate != "local") {
        Error(*node,
              fmt::format(
                  "Compiler attribute coordinate={} is not supported yet.",
                  coordinate));
        // No need to return here. Passing this point will result in a wrong
        // model, but not in parser crashes.
      }
    }
    std::string angle;
    if (ParseStringAttribute(node, "angle", &angle)) {
      if (angle == "degree") {
        angle_ = kDegree;
      } else if (angle == "radian") {
        angle_ = kRadian;
      } else {
        Warning(
            *node,
            fmt::format(
                "Unknown value {} for the compiler `angle` attribute will be "
                "ignored.  The existing value of `angle={}` will remain "
                "unchanged.",
                angle, (angle_ == kDegree) ? "degree" : "radian"));
      }
    }

    std::string assetdir;
    if (ParseStringAttribute(node, "assetdir", &assetdir)) {
      // assetdir sets both meshdir and texturedir, but texturedir is not
      // supported.
      meshdir_ = assetdir;
    }
    std::string meshdir;
    if (ParseStringAttribute(node, "meshdir", &meshdir)) {
      // meshdir takes priority over assetdir.
      meshdir_ = meshdir;
    }

    std::string eulerseq;
    if (ParseStringAttribute(node, "eulerseq", &eulerseq)) {
      if (eulerseq.size() != 3 ||
          eulerseq.find_first_not_of("xyzXYZ") != std::string::npos) {
        Error(
            *node,
            fmt::format(
                "Illegal value '{}' for the eulerseq in {}. Valid eulerseq are "
                "exactly three characters from the set [x,y,z,X,Y,Z]",
                eulerseq, node->Name()));

      } else {
        eulerseq_ = eulerseq;
      }
    }

    bool flag;
    switch (node->QueryBoolAttribute("inertiafromgeom", &flag)) {
      case tinyxml2::XML_SUCCESS:
        inertia_from_geom_ = flag ? kTrue : kFalse;
        break;
      case tinyxml2::XML_WRONG_ATTRIBUTE_TYPE:
        if (std::string(node->Attribute("inertiafromgeom")) == "auto") {
          inertia_from_geom_ = kAuto;
          break;
        }
        Warning(
            *node,
            fmt::format(
                "The attribute 'inertiafromgeom' found in a '{}' tag cannot be "
                "interpreted as a boolean nor 'auto' and will be ignored.",
                node->Name()));
        break;
      default:
        // Ok. No attribute to set.
        break;
    }

    // Unsupported attributes are listed in the order from the MuJoCo docs:
    // https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler
    WarnUnsupportedAttribute(*node, "boundmass");
    WarnUnsupportedAttribute(*node, "boundinertia");
    WarnUnsupportedAttribute(*node, "settotalmass");
    WarnUnsupportedAttribute(*node, "balanceinertia");
    WarnUnsupportedAttribute(*node, "strippath");
    WarnUnsupportedAttribute(*node, "fitaabb");
    WarnUnsupportedAttribute(*node, "texturedir");
    WarnUnsupportedAttribute(*node, "discardvisual");
    LogIgnoredAttribute(*node, "usethread");  // specific to MuJoCo runtime.
    WarnUnsupportedAttribute(*node, "fusestatic");
    WarnUnsupportedAttribute(*node, "alignfree");
    WarnUnsupportedAttribute(*node, "inertiagrouprange");
    WarnUnsupportedElement(*node, "lengthrange");
  }

  void ParseContact(XMLElement* node) {
    if (!plant_->geometry_source_is_registered()) {
      // No need to parse contacts if there is no SceneGraph registered.
      return;
    }

    geometry::CollisionFilterManager manager =
        scene_graph_->collision_filter_manager();
    const geometry::SceneGraphInspector<double>& inspector =
        scene_graph_->model_inspector();
    const auto geom_ids = inspector.GetGeometryIds(
        geometry::GeometrySet(inspector.GetAllGeometryIds()),
        geometry::Role::kProximity);

    // "Frame group" is the scene graph analog of model instance index;
    // MultibodyPlant guarantees that the numeric values match.
    int frame_group = model_instance_;

    auto geom_id_from_name = [&inspector, &geom_ids,
                              frame_group](const std::string& name) {
      for (GeometryId id : geom_ids) {
        // Only match geometry loaded from the current model instance.
        int candidate_frame_group =
            inspector.GetFrameGroup(inspector.GetFrameId(id));
        if (candidate_frame_group != frame_group) {
          continue;
        }

        // MultibodyPlant Register__Geometry methods automatically change the
        // geometry name to model_instance_name::geometry_name (in
        // MultibodyPlant::GetScopedName). Cope with that change here.
        const std::string candidate_name = inspector.GetName(id);
        if (candidate_name.ends_with(name)) {
          return id;
        }
      }
      return GeometryId();
    };

    for (XMLElement* pair_node = node->FirstChildElement("pair"); pair_node;
         pair_node = pair_node->NextSiblingElement("pair")) {
      std::string class_name;
      if (!ParseStringAttribute(node, "class", &class_name)) {
        class_name = "main";
      }
      ApplyDefaultAttributes("pair", class_name, pair_node);

      std::string geom1, geom2;
      if (!ParseStringAttribute(pair_node, "geom1", &geom1) ||
          !ParseStringAttribute(pair_node, "geom2", &geom2)) {
        Warning(*node,
                "contact pair node does not have required geom1 and/or geom2 "
                "attributes, so will be ignored.");
        continue;
      }

      GeometryId geom1_id = geom_id_from_name(geom1),
                 geom2_id = geom_id_from_name(geom2);
      if (!geom1_id.is_valid()) {
        Warning(
            *node,
            fmt::format(
                "contact pair specified unknown geom1 {} and will be ignored.",
                geom1));
        continue;
      }
      if (!geom2_id.is_valid()) {
        Warning(
            *node,
            fmt::format(
                "contact pair specified unknown geom2 {} and will be ignored.",
                geom2));
        continue;
      }

      // Unsupported attributes are listed in the order from the MuJoCo docs:
      // https://mujoco.readthedocs.io/en/stable/XMLreference.html#contact-pair
      // Silently ignored attributes a very specific to the MuJoCo solver.
      WarnUnsupportedAttribute(*pair_node, "condim");
      WarnUnsupportedAttribute(*pair_node, "friction");
      LogIgnoredAttribute(*pair_node, "solref");
      LogIgnoredAttribute(*pair_node, "solimp");
      LogIgnoredAttribute(*pair_node, "solreffriction");
      LogIgnoredAttribute(*pair_node, "margin");
      LogIgnoredAttribute(*pair_node, "gap");

      if (plant_->get_adjacent_bodies_collision_filters()) {
        // If true, then Finalize will declare a collision filter which
        // excludes joint parent/child bodies. Check that we don't have any
        // joints that would overwrite this setting during Finalize (Note that
        // all joints have already been parsed.)
        const BodyIndex body1_index =
            plant_->GetBodyFromFrameId(inspector.GetFrameId(geom1_id))->index();
        const BodyIndex body2_index =
            plant_->GetBodyFromFrameId(inspector.GetFrameId(geom2_id))->index();
        for (const auto& joint_index :
             plant_->GetJointIndices(model_instance_)) {
          const Joint<double>& joint = plant_->get_joint(joint_index);
          if ((joint.parent_body().index() == body1_index &&
               joint.child_body().index() == body2_index) ||
              (joint.parent_body().index() == body2_index &&
               (joint.child_body().index() == body1_index))) {
            Warning(*node,
                    fmt::format(
                        "This mjcf specified a contact pair with geom1 {} and "
                        "are on adjacent bodies (connected by a joint). A "
                        "collision filter excluding these bodies will be added "
                        "during MultibodyPlant::Finalize(), which will "
                        "overwrite the collision filter specified in this "
                        "file. To avoid this you must call "
                        "MultibodyPlant::set_adjacent_bodies_collision_filters("
                        "), and understand the implications.",
                        geom1, geom2));
          }
        }
      }

      manager.Apply(geometry::CollisionFilterDeclaration().AllowBetween(
          geometry::GeometrySet({geom1_id}),
          geometry::GeometrySet({geom2_id})));
    }

    for (XMLElement* exclude_node = node->FirstChildElement("exclude");
         exclude_node;
         exclude_node = exclude_node->NextSiblingElement("exclude")) {
      std::string body1, body2;
      if (!ParseStringAttribute(exclude_node, "body1", &body1) ||
          !ParseStringAttribute(exclude_node, "body2", &body2)) {
        Warning(
            *node,
            "contact exclude node does not have required body1 and/or body2 "
            "attributes, so will be ignored.");
        continue;
      }

      geometry::FrameId fid1 = plant_->GetBodyFrameIdOrThrow(
          plant_->GetBodyByName(body1, model_instance_).index());
      geometry::FrameId fid2 = plant_->GetBodyFrameIdOrThrow(
          plant_->GetBodyByName(body2, model_instance_).index());
      manager.Apply(geometry::CollisionFilterDeclaration().ExcludeBetween(
          geometry::GeometrySet(fid1), geometry::GeometrySet(fid2)));
    }
  }

  void ParseEquality(XMLElement* node) {
    // Per the mujoco docs: "The actual equality constraints have types
    // depending on the sub-element used to define them. However here we are
    // setting attributes common to all equality constraint types, which is why
    // we do not make a distinction between types." This helper method applies
    // the default_equality to each sub-element.
    auto apply_defaults = [&](XMLElement* node_on_which_to_apply) {
      std::string class_name;
      if (!ParseStringAttribute(node_on_which_to_apply, "class", &class_name)) {
        class_name = "main";
      }
      ApplyDefaultAttributes("equality", class_name, node_on_which_to_apply);
    };

    for (XMLElement* connect_node = node->FirstChildElement("connect");
         connect_node;
         connect_node = connect_node->NextSiblingElement("connect")) {
      apply_defaults(connect_node);

      // Per the mujoco docs: connect can be specified with either "body1",
      // "anchor", (both required) and optionally "body2" OR "site1" and
      // "site2" (both required).
      if (connect_node->Attribute("body1") &&
          connect_node->Attribute("site1")) {
        Error(*connect_node,
              "connect node must specify either body1 and anchor OR site1 and "
              "site2, but not both.");
        continue;
      }

      std::string body1;
      if (ParseStringAttribute(connect_node, "body1", &body1)) {
        // Then "body1", "anchor", and optionally "body2".
        Vector3d p_AP;
        if (!ParseVectorAttribute(connect_node, "anchor", &p_AP)) {
          Error(*node,
                "connect specified body1 but does not have the required anchor "
                "attribute.");
          continue;
        }
        if (!plant_->HasBodyNamed(body1, model_instance_)) {
          Error(*node,
                fmt::format("connect specified body1: {} but no body with that "
                            "name exists in model instance: {}",
                            body1,
                            plant_->GetModelInstanceName(model_instance_)));
          continue;
        }
        const RigidBody<double>& body_A =
            plant_->GetBodyByName(body1, model_instance_);
        const RigidBody<double>* body_B = &plant_->world_body();
        std::string body2;
        if (ParseStringAttribute(connect_node, "body2", &body2)) {
          if (!plant_->HasBodyNamed(body2, model_instance_)) {
            Error(*node,
                  fmt::format(
                      "connect specified body2: {} but no body with that "
                      "name exists in model instance: {}",
                      body2, plant_->GetModelInstanceName(model_instance_)));
            continue;
          }
          body_B = &plant_->GetBodyByName(body2, model_instance_);
        }
        plant_->AddBallConstraint(body_A, p_AP, *body_B);
      } else {
        std::string site1, site2;
        if (!ParseStringAttribute(connect_node, "site1", &site1) ||
            !ParseStringAttribute(connect_node, "site2", &site2)) {
          Error(*connect_node,
                "connect must specify body1 and anchor OR site1 and site2.");
          continue;
        } else {
          Warning(*connect_node,
                  "connect node uses the site1 and site2 specification, which "
                  "is not supported yet. This constraint will be ignored.");
          continue;
        }
      }

      // Unsupported attributes are listed in the order from the MuJoCo docs:
      // https://mujoco.readthedocs.io/en/stable/XMLreference.html#equality-connect
      // The ignored attributes are specific to the MuJoCo solver.
      WarnUnsupportedAttribute(*connect_node, "active");
      LogIgnoredAttribute(*connect_node, "solref");
      LogIgnoredAttribute(*connect_node, "solimp");
    }

    // TODO(russt): "weld" constraints are already supported by MultibodyPlant
    // and should be easy to add.

    // Unsupported elements are listed in the order from the MuJoCo docs:
    // https://mujoco.readthedocs.io/en/stable/XMLreference.html#equality
    WarnUnsupportedElement(*node, "weld");
    WarnUnsupportedElement(*node, "joint");
    WarnUnsupportedElement(*node, "tendon");
    WarnUnsupportedElement(*node, "flex");
    WarnUnsupportedElement(*node, "distance");  // removed in MuJoCo 2.2.2
  }

  // Updates node by recursively replacing any <include> elements under it with
  // the children of the named file's root element.
  void ExpandIncludeTags(XMLElement* node,
                         const std::filesystem::path& parent_mjcf_path) {
    DRAKE_DEMAND(node != nullptr);

    // Process the current node if it's an <include> tag.
    if (std::string(node->Value()) == "include") {
      std::string file;
      if (!ParseStringAttribute(node, "file", &file)) {
        Error(*node, "<include> tag without file attribute.");
        return;
      }

      // From the MJCF docs: "The name of the XML file to be included. The file
      // location is relative to the directory of the main MJCF file. If the
      // file is not in the same directory, it should be prefixed with a
      // relative path."
      std::filesystem::path filename = parent_mjcf_path / file;
      filename = std::filesystem::absolute(filename);
      log()->debug("Processing included file: {}", filename.string());

      XMLDocument include_doc;
      if (include_doc.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
        Error(*node, fmt::format("Failed to load <include> file at path {}.",
                                 filename.string()));
        return;
      }

      XMLElement* include_root = include_doc.RootElement();
      if (!include_root) {
        Error(*node, fmt::format("Included file {} has no root element.",
                                 filename.string()));
        return;
      }

      // Insert the children of the root element of the included file.
      XMLDocument* xml_doc = node->GetDocument();
      XMLElement* parent = node->Parent()->ToElement();
      XMLElement* node_in_parent = node;

      XMLElement* child = include_root->FirstChildElement();
      while (child) {
        // Insert the child node after the include node (or after the last
        // inserted node).
        parent->InsertAfterChild(node_in_parent, child->DeepClone(xml_doc));
        node_in_parent = node_in_parent->NextSiblingElement();
        child = child->NextSiblingElement();
      }

      return;
    }

    // Recurse on child elements.
    XMLElement* child = node->FirstChildElement();
    while (child) {
      ExpandIncludeTags(child, parent_mjcf_path);
      XMLElement* to_delete =
          (std::string(child->Value()) == "include") ? child : nullptr;
      child = child->NextSiblingElement();

      if (to_delete) {
        // Remove the include node from the main document.
        node->GetDocument()->DeleteNode(to_delete);
      }
    }
  }

  // Assets without an absolute path are referenced relative to the "main MJCF
  // model file" path, `main_mjcf_path`.
  std::pair<std::optional<ModelInstanceIndex>, std::string> Parse(
      const std::string& model_name_in,
      const std::optional<std::string>& parent_model_name,
      std::optional<ModelInstanceIndex> merge_into_model_instance,
      XMLDocument* xml_doc, const std::filesystem::path& main_mjcf_path) {
    main_mjcf_path_ = main_mjcf_path;

    XMLElement* node = xml_doc->FirstChildElement("mujoco");
    if (!node) {
      Error(*xml_doc, "ERROR: XML does not contain a mujoco tag.");
      return {};
    }

    // Per the mjcf docs, the include tags are processed purely lexically.
    ExpandIncludeTags(xml_doc->RootElement(), main_mjcf_path_);

    std::string model_name = model_name_in;
    if (model_name.empty() &&
        !ParseStringAttribute(node, "model", &model_name)) {
      Error(*node,
            "ERROR: Your robot must have a name attribute or a model name "
            "must be specified.");
      return {};
    }

    if (!merge_into_model_instance.has_value()) {
      model_name = MakeModelName(model_name, parent_model_name, workspace_);
      model_instance_ = plant_->AddModelInstance(model_name);
    } else {
      model_instance_ = *merge_into_model_instance;
    }

    // Parse the compiler parameters.
    for (XMLElement* compiler_node = node->FirstChildElement("compiler");
         compiler_node;
         compiler_node = compiler_node->NextSiblingElement("compiler")) {
      ParseCompiler(compiler_node);
    }

    // Parse the options.
    for (XMLElement* option_node = node->FirstChildElement("option");
         option_node; option_node = option_node->NextSiblingElement("option")) {
      ParseOption(option_node);
    }

    // Parse the defaults.
    for (XMLElement* default_node = node->FirstChildElement("default");
         default_node;
         default_node = default_node->NextSiblingElement("default")) {
      ParseDefault(default_node);
    }

    // Parse the assets. This must happen after parsing the defaults (which
    // could set the assetdir).
    for (XMLElement* asset_node = node->FirstChildElement("asset"); asset_node;
         asset_node = asset_node->NextSiblingElement("asset")) {
      ParseAsset(asset_node);
    }

    // Parses the model's world link elements.
    for (XMLElement* link_node = node->FirstChildElement("worldbody");
         link_node; link_node = link_node->NextSiblingElement("worldbody")) {
      ParseWorldBody(link_node);
    }

    // Parses the model's link elements.
    for (XMLElement* link_node = node->FirstChildElement("body"); link_node;
         link_node = link_node->NextSiblingElement("body")) {
      ParseBody(link_node, plant_->world_body(), RigidTransformd());
    }

    // Parses the model's actuator elements.
    for (XMLElement* actuator_node = node->FirstChildElement("actuator");
         actuator_node;
         actuator_node = actuator_node->NextSiblingElement("actuator")) {
      ParseActuator(actuator_node);
    }

    // Parses the model's contact elements.
    for (XMLElement* contact_node = node->FirstChildElement("contact");
         contact_node;
         contact_node = contact_node->NextSiblingElement("contact")) {
      ParseContact(contact_node);
    }

    // Parses the model's equality elements.
    for (XMLElement* equality_node = node->FirstChildElement("equality");
         equality_node;
         equality_node = equality_node->NextSiblingElement("equality")) {
      ParseEquality(equality_node);
    }

    // Unsupported elements are listed in the order from the MuJoCo docs:
    // https://mujoco.readthedocs.io/en/stable/XMLreference.html#mjcf-reference
    LogIgnoredElement(*node,
                      "size");  // specific memory allocation for MuJoCo solver.
    WarnUnsupportedElement(*node, "statistic");
    WarnUnsupportedElement(*node, "deformable");
    WarnUnsupportedElement(*node, "tendon");
    WarnUnsupportedElement(*node, "sensor");
    WarnUnsupportedElement(*node, "keyframe");
    WarnUnsupportedElement(*node, "visual");
    // custom variables for MuJoCo; we don't plan to have a Drake equivalent.
    LogIgnoredElement(*node, "custom");
    WarnUnsupportedElement(*node, "extension");

    return std::make_pair(model_instance_, model_name);
  }

  void Warning(const XMLNode& location, std::string message) const {
    diagnostic_.Warning(location, std::move(message));
  }

  void Error(const XMLNode& location, std::string message) const {
    diagnostic_.Error(location, std::move(message));
  }

  // Warn about documented Mujoco elements ignored by Drake.
  void WarnUnsupportedElement(const XMLElement& node, const std::string& tag) {
    diagnostic_.WarnUnsupportedElement(node, tag);
  }

  // Warn about documented Mujoco attributes ignored by Drake.
  void WarnUnsupportedAttribute(const XMLElement& node,
                                const std::string& attribute) {
    diagnostic_.WarnUnsupportedAttribute(node, attribute);
  }

  void LogIgnoredAttribute(const XMLElement& node,
                           const std::string& attribute) {
    log()->debug("Ignored attribute: {} on element: {}", attribute,
                 node.Value());
  }

  void LogIgnoredElement(const XMLElement& node, const std::string& element) {
    log()->debug("Ignored element: {} in element: {}", element, node.Value());
  }

 private:
  const ParsingWorkspace& workspace_;
  TinyXml2Diagnostic diagnostic_;
  systems::DiagramBuilder<double>* const builder_;
  MultibodyPlant<double>* const plant_;
  geometry::SceneGraph<double>* const scene_graph_;
  ModelInstanceIndex model_instance_{};
  std::filesystem::path main_mjcf_path_{};
  bool autolimits_{true};
  enum Angle { kRadian, kDegree };
  Angle angle_{kDegree};
  // The defaults_ map is from (element name, class name) to the XMLElement.
  std::map<std::pair<std::string, std::string>, XMLElement*> defaults_{};
  enum InertiaFromGeometry { kFalse, kTrue, kAuto };
  InertiaFromGeometry inertia_from_geom_{kAuto};
  std::map<std::string, XMLElement*> material_{};
  std::optional<std::filesystem::path> meshdir_{};
  std::string eulerseq_{"xyz"};
  std::map<std::string, std::unique_ptr<geometry::Mesh>> mesh_{};
  // Spatial inertia of mesh assets assuming density = 1.
  std::map<std::string, SpatialInertia<double>> mesh_inertia_{};
  std::map<JointIndex, double> armature_{};
  int num_cameras_{0};
};

std::pair<std::optional<ModelInstanceIndex>, std::string>
AddOrMergeModelFromMujocoXml(
    const DataSource& data_source, const std::string& model_name_in,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace,
    std::optional<ModelInstanceIndex> merge_into_model_instance) {
  DRAKE_THROW_UNLESS(!workspace.plant->is_finalized());

  TinyXml2Diagnostic diag(&workspace.diagnostic, &data_source);
  XMLDocument xml_doc;
  std::filesystem::path path{};
  if (data_source.IsFilename()) {
    xml_doc.LoadFile(data_source.filename().c_str());
    if (xml_doc.ErrorID()) {
      diag.Error(xml_doc,
                 fmt::format("Failed to parse XML file {}:\n{}",
                             data_source.filename(), xml_doc.ErrorName()));
      return {};
    }
    path = data_source.filename();
    path.remove_filename();
  } else {
    xml_doc.Parse(data_source.contents().c_str());
    if (xml_doc.ErrorID()) {
      diag.Error(xml_doc, fmt::format("Failed to parse XML string: {}",
                                      xml_doc.ErrorName()));
      return {};
    }
    path = std::filesystem::current_path();
  }

  MujocoParser parser(workspace, data_source);
  return parser.Parse(model_name_in, parent_model_name,
                      merge_into_model_instance, &xml_doc, path);
}
}  // namespace

std::optional<ModelInstanceIndex> AddModelFromMujocoXml(
    const DataSource& data_source, const std::string& model_name_in,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  return AddOrMergeModelFromMujocoXml(data_source, model_name_in,
                                      parent_model_name, workspace,
                                      std::nullopt)
      .first;
}

MujocoParserWrapper::MujocoParserWrapper() {}

MujocoParserWrapper::~MujocoParserWrapper() {}

std::optional<ModelInstanceIndex> MujocoParserWrapper::AddModel(
    const DataSource& data_source, const std::string& model_name,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  return AddModelFromMujocoXml(data_source, model_name, parent_model_name,
                               workspace);
}

std::string MujocoParserWrapper::MergeModel(
    const DataSource& data_source, const std::string& model_name,
    ModelInstanceIndex merge_into_model_instance,
    const ParsingWorkspace& workspace) {
  return AddOrMergeModelFromMujocoXml(data_source, model_name, std::nullopt,
                                      workspace, merge_into_model_instance)
      .second;
}

std::vector<ModelInstanceIndex> MujocoParserWrapper::AddAllModels(
    const DataSource& data_source,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  auto maybe_model = AddModel(data_source, {}, parent_model_name, workspace);
  if (maybe_model.has_value()) {
    return {*maybe_model};
  } else {
    return {};
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
