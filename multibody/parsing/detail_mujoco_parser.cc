#include "drake/multibody/parsing/detail_mujoco_parser.h"

#include <algorithm>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <drake_vendor/tinyxml2.h>
#include <fmt/format.h>

#include "drake/geometry/proximity/meshing_utilities.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/detail_tinyxml.h"
#include "drake/multibody/parsing/scoped_names.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/weld_joint.h"

namespace drake {
namespace multibody {
namespace internal {

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using geometry::GeometryInstance;
using math::RigidTransformd;
using math::RotationMatrixd;
using tinyxml2::XMLAttribute;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

namespace {

// For tags that exist in the MuJoCo XML schema, but are not (yet) supported
// here, we print a warning to the console.  This method checks the node for
// the child element, and prints the warning if any occurrences exist.
void WarnUnsupportedElement(const XMLElement& node, const std::string& tag) {
  if (node.FirstChildElement(tag.c_str())) {
    drake::log()->warn(
        "The tag '{}' found as a child of '{}' is currently unsupported and "
        "will be ignored.",
        tag, node.Name());
  }
}

// For tags that exist in the MuJoCo XML schema, but are not (yet) supported
// here, we print a warning to the console.  This method checks the node for
// the child element, and prints the warning if any occurrences exist.
void WarnUnsupportedAttribute(const XMLElement& node,
                              const std::string& attribute) {
  if (node.Attribute(attribute.c_str())) {
    drake::log()->warn(
        "The attribute '{}' found in a {} tag is currently unsupported and "
        "will be ignored.",
        attribute, node.Name());
  }
}

// Any attributes from `default` that are not specified in `node` will be added
// to `node`.
void ApplyDefaultAttributes(const XMLElement& default_node, XMLElement* node) {
  for (const XMLAttribute* default_attr = default_node.FirstAttribute();
       default_attr != nullptr; default_attr = default_attr->Next()) {
    if (!node->Attribute(default_attr->Name())) {
      node->SetAttribute(default_attr->Name(), default_attr->Value());
    }
  }
}

class MujocoParser {
 public:
  explicit MujocoParser(MultibodyPlant<double>* plant) : plant_{plant} {}

  RigidTransformd ParseTransform(
      XMLElement* node, const RigidTransformd& X_default = RigidTransformd{}) {
    Vector3d pos(X_default.translation());
    ParseVectorAttribute(node, "pos", &pos);

    // Check that only one of the orientation variants are supplied:
    const int num_orientation_attrs =
        (node->Attribute("quat") != nullptr ? 1 : 0)
        + (node->Attribute("axisangle") != nullptr ? 1 : 0)
        + (node->Attribute("euler") != nullptr ? 1 : 0)
        + (node->Attribute("xyaxes") != nullptr ? 1 : 0)
        + (node->Attribute("zaxis") != nullptr ? 1 : 0);
    if (num_orientation_attrs > 1) {
      throw std::logic_error(fmt::format(
          "Element {} has more than one orientation attribute specified "
          "(perhaps through defaults). There must be no more than one instance "
          "of `quat`, `axisangle`, `euler`, `xyaxes`, or `zaxis`.",
          node->Name()));
    }

    Vector4d quat;  // MuJoCo uses w,x,y,z order.
    if (ParseVectorAttribute(node, "quat", &quat)) {
      return RigidTransformd(
          Eigen::Quaternion<double>(quat[0], quat[1], quat[2], quat[3]), pos);
    }

    Vector4d axisangle;
    if (ParseVectorAttribute(node, "axisangle", &axisangle)) {
      if (angle_ == kDegree) {
        axisangle[3] *= (M_PI / 180.0);
      }
      return RigidTransformd(
          Eigen::AngleAxis<double>(axisangle[3], axisangle.head<3>()), pos);
    }

    Vector3d euler;
    if (ParseVectorAttribute(node, "euler", &euler)) {
      if (angle_ == kDegree) {
        euler *= (M_PI / 180.0);
      }
      // Default is extrinsic xyz.  See eulerseq compiler option.
      return RigidTransformd(math::RollPitchYawd(euler), pos);
    }

    Vector6d xyaxes;
    if (ParseVectorAttribute(node, "xyaxes", &xyaxes)) {
      Matrix3d R;
      R.col(0) = xyaxes.head<3>();
      R.col(1) = xyaxes.tail<3>();
      R.col(2) = xyaxes.head<3>().cross(xyaxes.tail<3>());
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

  void ParseMotor(XMLElement* node) {
    std::string name;
    if (!ParseStringAttribute(node, "name", &name)) {
      // Use "motor#" as the default actuator name.
      name = fmt::format("motor{}", plant_->num_actuators());
    }

    std::string joint_name;
    if (!ParseStringAttribute(node, "joint", &joint_name)) {
      drake::log()->warn(
          "The motor '{}' does not use the 'joint' transmission specification. "
          "Currently only the 'joint' attribute is supported.  This motor will "
          "be ignored.",
          name);
      return;
    }

    // Parse effort limits.
    double effort_limit = std::numeric_limits<double>::infinity();
    bool ctrl_limited = node->BoolAttribute("ctrllimited", false);
    if (ctrl_limited) {
      Vector2d ctrl_range;
      if (ParseVectorAttribute(node, "ctrlrange", &ctrl_range)) {
        if (ctrl_range[0] > ctrl_range[1]) {
          drake::log()->warn(
              "The motor '{}' specified a ctrlrange attribute where lower "
              "limit > upper limit; these limits will be ignored.",
              name);
        } else {
          effort_limit = std::max(ctrl_range[1], -ctrl_range[0]);
          if (ctrl_range[0] != ctrl_range[1]) {
            drake::log()->warn(
                "The motor '{}' specified a ctrlrange attribute where lower "
                "limit != upper limit.  Asymmetrical effort limits are not "
                "supported yet, so the larger of the values {} will be used.",
                name, effort_limit);
          }
        }
      }
    }
    bool force_limited = node->BoolAttribute("forcelimited", false);
    if (force_limited) {
      // For a motor, force limits are the same as control limits, so we take
      // the min of the two.
      Vector2d force_range;
      if (ParseVectorAttribute(node, "forcerange", &force_range)) {
        if (force_range[0] > force_range[1]) {
          drake::log()->warn(
              "The motor '{}' specified a forcerange attribute where lower "
              "limit > upper limit; these limits will be ignored.",
              name);
        } else {
          effort_limit =
              std::min(effort_limit, std::max(force_range[1], -force_range[0]));
          if (force_range[0] != force_range[1]) {
            drake::log()->warn(
                "The motor '{}' specified a forcerange attribute where lower "
                "limit != upper limit.  Asymmetrical effort limits are not "
                "supported yet, so the larger of the values {} will be used.",
                name, std::max(force_range[1], -force_range[0]));
          }
        }
      }
    }

    WarnUnsupportedAttribute(*node, "jointinparent");
    WarnUnsupportedAttribute(*node, "tendon");
    WarnUnsupportedAttribute(*node, "site");

    WarnUnsupportedAttribute(*node, "class");
    WarnUnsupportedAttribute(*node, "group");
    WarnUnsupportedAttribute(*node, "lengthrange");
    WarnUnsupportedAttribute(*node, "gear");
    WarnUnsupportedAttribute(*node, "cranklength");
    WarnUnsupportedAttribute(*node, "cranksite");
    WarnUnsupportedAttribute(*node, "slidersite");
    WarnUnsupportedAttribute(*node, "user");

    plant_->AddJointActuator(
        name, plant_->GetJointByName(joint_name, model_instance_),
        effort_limit);
  }

  void ParseActuator(XMLElement* node) {
    for (XMLElement* motor_node = node->FirstChildElement("motor"); motor_node;
         motor_node = motor_node->NextSiblingElement("motor")) {
      ParseMotor(motor_node);
    }
    WarnUnsupportedElement(*node, "include");
    WarnUnsupportedElement(*node, "general");
    WarnUnsupportedElement(*node, "position");
    WarnUnsupportedElement(*node, "velocity");
    WarnUnsupportedElement(*node, "cylinder");
    WarnUnsupportedElement(*node, "muscle");
  }

  void ParseJoint(XMLElement* node, const RigidBody<double>& parent,
                  const RigidBody<double>& child, const RigidTransformd& X_WC,
                  const RigidTransformd& X_PC) {
    std::string name;
    if (!ParseStringAttribute(node, "name", &name)) {
      // Use "parent-body" as the default joint name.
      name = fmt::format("{}-{}", parent.name(), child.name());
    }

    Vector3d pos = Vector3d::Zero();
    ParseVectorAttribute(node, "pos", &pos);
    const RigidTransformd X_PJ(pos);
    const RigidTransformd X_CJ = X_PC.InvertAndCompose(X_PJ);

    Vector3d axis = Vector3d::UnitZ();
    ParseVectorAttribute(node, "axis", &axis);

    double damping{0.0};
    ParseScalarAttribute(node, "damping", &damping);

    std::string type;
    if (!ParseStringAttribute(node, "type", &type)) {
      type = "hinge";
    }

    bool limited = node->BoolAttribute("limited", false);
    Vector2d range(0.0, 0.0);
    ParseVectorAttribute(node, "range", &range);

    if (type == "free") {
      if (damping != 0.0) {
        drake::log()->warn(
            "Damping was specified for the 'free' joint {}, but is not "
            "supported for free bodies.",
            name);
      }
      plant_->SetDefaultFreeBodyPose(child, X_WC);
    } else if (type == "ball") {
      plant_->AddJoint<BallRpyJoint>(name, parent, X_PJ, child, X_CJ, damping);
      if (limited) {
        WarnUnsupportedAttribute(*node, "range");
      }
    } else if (type == "slide") {
      JointIndex index =
          plant_
              ->AddJoint<PrismaticJoint>(
                  name, parent, X_PJ, child, X_CJ, axis,
                  -std::numeric_limits<double>::infinity(),
                  std::numeric_limits<double>::infinity(), damping)
              .index();
      if (limited) {
        plant_->get_mutable_joint(index).set_position_limits(
            Vector1d{range[0]}, Vector1d{range[1]});
      }
    } else if (type == "hinge") {
      JointIndex index = plant_
                             ->AddJoint<RevoluteJoint>(
                                 name, parent, X_PJ, child, X_CJ, axis, damping)
                             .index();
      if (limited) {
        if (angle_ == kDegree) {
          range *= (M_PI / 180.0);
        }
        plant_->get_mutable_joint(index).set_position_limits(
            Vector1d{range[0]}, Vector1d{range[1]});
      }
    } else {
      throw std::logic_error("Unknown joint type " + type);
    }

    WarnUnsupportedAttribute(*node, "class");
    WarnUnsupportedAttribute(*node, "group");
    WarnUnsupportedAttribute(*node, "springdamper");
    WarnUnsupportedAttribute(*node, "solreflimit");
    WarnUnsupportedAttribute(*node, "solimplimit");
    WarnUnsupportedAttribute(*node, "solreffriction");
    WarnUnsupportedAttribute(*node, "solimpfriction");
    WarnUnsupportedAttribute(*node, "stiffness");
    WarnUnsupportedAttribute(*node, "margin");
    WarnUnsupportedAttribute(*node, "ref");
    WarnUnsupportedAttribute(*node, "springref");
    // TODO(joemasterjohn): Parse and stash "armature" tag for the appropriate
    // JointActuator attached to this joint.
    WarnUnsupportedAttribute(*node, "armature");
    WarnUnsupportedAttribute(*node, "frictionloss");
    WarnUnsupportedAttribute(*node, "user");
  }

  SpatialInertia<double> ParseInertial(XMLElement* node) {
    // We use F to denote the "inertial frame" in the MujoCo documentation.  B
    // is the body frame.
    RigidTransformd X_BF = ParseTransform(node);
    double mass;
    if (!ParseScalarAttribute(node, "mass", &mass)) {
      throw std::logic_error(
          "The inertial tag must include the mass attribute.");
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
        throw std::logic_error(
            "The inertial tag must include either the diaginertia or "
            "fullinertia attribute.");
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
    SpatialInertia<double> M_GBo_B{};
  };

  MujocoGeometry ParseGeometry(XMLElement* node, int num_geom,
                               bool compute_inertia,
                               const std::string& child_class = "") {
    MujocoGeometry geom;

    if (!ParseStringAttribute(node, "name", &geom.name)) {
      // Use "geom#" as the default body name.
      geom.name = fmt::format("geom{}", num_geom);
    }

    std::string class_name;
    if (!ParseStringAttribute(node, "class", &class_name)) {
      class_name = child_class.empty() ? "main" : child_class;
    }
    if (default_geometry_.count(class_name) > 0) {
      // TODO(russt): Add a test case covering childclass/default nesting once
      // the body element is supported.
      ApplyDefaultAttributes(*default_geometry_.at(class_name), node);
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
      size = ConvertToDoubles(size_attr);
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

    multibody::UnitInertia<double> unit_M_GG_G;
    std::string mesh;
    if (type == "plane") {
      // We interpret the MuJoCo infinite plane as a half-space.
      geom.shape = std::make_unique<geometry::HalfSpace>();
      // No inertia; can only be static geometry.
    } else if (type == "sphere") {
      if (size.size() < 1) {
        throw std::logic_error(
            "The size attribute for sphere geom must have at least one "
            "element.");
      }
      geom.shape = std::make_unique<geometry::Sphere>(size[0]);
      unit_M_GG_G = multibody::UnitInertia<double>::SolidSphere(size[0]);
    } else if (type == "capsule") {
      if (has_fromto) {
        if (size.size() < 1) {
          throw std::logic_error(
              "The size attribute for capsule geom using fromto must have at "
              "least one element.");
        }
        double length = (fromto.head<3>() - fromto.tail<3>()).norm();
        geom.shape = std::make_unique<geometry::Capsule>(size[0], length);
        unit_M_GG_G =
            multibody::UnitInertia<double>::SolidCapsule(size[0], length);

      } else {
        if (size.size() < 2) {
          throw std::logic_error(
              "The size attribute for capsule geom must have at least two "
              "elements.");
        }
        geom.shape = std::make_unique<geometry::Capsule>(size[0], 2 * size[1]);
        unit_M_GG_G =
            multibody::UnitInertia<double>::SolidCapsule(size[0], 2 * size[1]);
      }
    } else if (type == "ellipsoid") {
      if (has_fromto) {
        // The specification is not properly documented in the XML schema.
        drake::log()->warn(
            "The fromto tag for ellipsoid is currently unsupported; geom {} "
            "will be ignored.",
            geom.name);
        return geom;
      } else {
        if (size.size() < 3) {
          throw std::logic_error(
              "The size attribute for ellipsoid geom must have at least three "
              "elements.");
        }
        geom.shape =
            std::make_unique<geometry::Ellipsoid>(size[0], size[1], size[2]);
        unit_M_GG_G = multibody::UnitInertia<double>::SolidEllipsoid(
            size[0], size[1], size[2]);
      }
    } else if (type == "cylinder") {
      if (has_fromto) {
        if (size.size() < 1) {
          throw std::logic_error(
              "The size attribute for capsule geom using fromto must have at "
              "least one element.");
        }
        double length = (fromto.head<3>() - fromto.tail<3>()).norm();
        geom.shape = std::make_unique<geometry::Cylinder>(size[0], length);
        unit_M_GG_G =
            multibody::UnitInertia<double>::SolidCylinder(size[0], length);
      } else {
        if (size.size() < 2) {
          throw std::logic_error(
              "The size attribute for cylinder geom must have at least two "
              "elements.");
        }
        geom.shape = std::make_unique<geometry::Cylinder>(size[0], 2 * size[1]);
        unit_M_GG_G =
            multibody::UnitInertia<double>::SolidCylinder(size[0], 2 * size[1]);
      }
    } else if (type == "box") {
      if (has_fromto) {
        // The specification is not properly documented in the XML schema.
        drake::log()->warn(
            "The fromto tag for box is currently unsupported; geom {} will be "
            "ignored.",
            geom.name);
        return geom;
      } else {
        if (size.size() < 3) {
          throw std::logic_error(
              "The size attribute for box geom must have at least three "
              "elements.");
        }
        geom.shape = std::make_unique<geometry::Box>(
            size[0] * 2.0, size[1] * 2.0, size[2] * 2.0);
        unit_M_GG_G = multibody::UnitInertia<double>::SolidBox(
            size[0] * 2.0, size[1] * 2.0, size[2] * 2.0);
      }
    } else if (type == "mesh") {
      if (!ParseStringAttribute(node, "mesh", &mesh)) {
        throw std::runtime_error(fmt::format(
            "geom {} specified type 'mesh', but did not set the mesh attribute",
            geom.name));
      }
      if (mesh_.count(mesh)) {
        geom.shape = mesh_.at(mesh)->Clone();
        if (compute_inertia) {
          // TODO(russt): Compute the proper unit inertia for the mesh geometry
          // (#18314). For now, we compute an OBB for the mesh and take the
          // unit inertia of the corresponding box.

          // At least so far, we have a surface mesh for every mesh.
          DRAKE_ASSERT(surface_mesh_.count(mesh) == 1);
          const geometry::TriangleSurfaceMesh<double>& surface_mesh =
              surface_mesh_.at(mesh);
          std::set<int> v;
          for (int i = 0; i < surface_mesh.num_vertices(); ++i) {
            v.insert(v.end(), i);
          }
          // TODO(russt): The ObbMaker should really make it easier to "use all
          // the vertices".
          const geometry::internal::Obb obb =
              geometry::internal::ObbMaker(surface_mesh, v).Compute();
          UnitInertia<double> unit_M_GBox_Box =
              multibody::UnitInertia<double>::SolidBox(
                  obb.half_width()[0] * 2.0, obb.half_width()[1] * 2.0,
                  obb.half_width()[2] * 2.0);
          unit_M_GG_G = unit_M_GBox_Box.ReExpress(obb.pose().rotation())
                            .ShiftFromCenterOfMass(-obb.pose().translation());
        }
      } else {
        log()->warn("geom {} specified unknown mesh {} and will be ignored",
                    geom.name, mesh);
        return geom;
      }
    } else if (type == "hfield") {
      log()->warn(
          "The geom type '{}' is currently unsupported and will be ignored.",
          type);
      return geom;
    } else {
      throw std::runtime_error(fmt::format("Unrecognized geom type {}", type));
    }

    if (type != "mesh") {
      // TODO(russt): Support the mesh tag for non-mesh geometry. (Presumably
      // this is how they specify different visual + collision geometry).
      WarnUnsupportedAttribute(*node, "mesh");
      /* From the MuJoCo docs: Note that mesh assets can also be referenced
      from other geom types, causing primitive shapes to be fitted; see below.
      The size is determined by the mesh asset and the geom size parameters are
      ignored. */
    }

    WarnUnsupportedAttribute(*node, "contype");
    WarnUnsupportedAttribute(*node, "conaffinity");
    WarnUnsupportedAttribute(*node, "condim");
    WarnUnsupportedAttribute(*node, "group");
    WarnUnsupportedAttribute(*node, "priority");

    std::string material;
    if (ParseStringAttribute(node, "material", &material)) {
      if (material_.count(material)) {
        XMLElement* material_node = material_.at(material);
        // Note: there are many material attributes that we do not support yet,
        // nor perhaps ever. Consider warning about them here (currently, it
        // seems like too much noise).
        ParseVectorAttribute(material_node, "rgba", &geom.rgba);
      } else {
        log()->warn("geom {} specified an unrecognized material {}", geom.name,
                    material);
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
      friction = ConvertToDoubles(friction_attr);
    }
    if (!friction.empty()) {
      // MuJoCo's friction specification is [sliding, torsional, rolling].  We
      // set the static and dynamic friction to `sliding`, and do not support
      // the other parameters.
      geom.friction = CoulombFriction(friction[0], friction[0]);
      if ((friction.size() > 1 && friction[1] != 0.0) ||
          (friction.size() > 2 && friction[2] != 0.0)) {
        drake::log()->warn(
            "The torsional and rolling friction specified in the friction "
            "attribute of {} are unsupported and will be ignored.",
            node->Name());
      }
    }

    WarnUnsupportedAttribute(*node, "solmix");
    WarnUnsupportedAttribute(*node, "solref");
    WarnUnsupportedAttribute(*node, "solimp");
    WarnUnsupportedAttribute(*node, "margin");
    WarnUnsupportedAttribute(*node, "gap");
    WarnUnsupportedAttribute(*node, "fitscale");
    WarnUnsupportedAttribute(*node, "user");

    if (compute_inertia) {
      double mass, density{1000};
      if (!ParseScalarAttribute(node, "mass", &mass)) {
        ParseScalarAttribute(node, "density", &density);
        if (type == "mesh") {
          // At least so far, we have a surface mesh for every mesh.
          DRAKE_ASSERT(surface_mesh_.count(mesh) == 1);
          mass = density *
                 geometry::internal::CalcEnclosedVolume(surface_mesh_.at(mesh));
        } else {
          mass = density * geometry::CalcVolume(*geom.shape);
        }
      }
      multibody::SpatialInertia<double> M_GG_G(mass, Vector3d::Zero(),
                                               unit_M_GG_G);
      geom.M_GBo_B = M_GG_G.ReExpress(geom.X_BG.rotation())
                         .Shift(-geom.X_BG.translation());
    }
    return geom;
  }

  void ParseBody(XMLElement* node, const RigidBody<double>& parent,
                 const RigidTransformd& X_WP,
                 const std::string& parent_class = "") {
    std::string body_name;
    if (!ParseStringAttribute(node, "name", &body_name)) {
      // Use "body#" as the default body name.
      body_name = fmt::format("body{}", plant_->num_bodies());
    }

    std::string child_class{};
    if (!ParseStringAttribute(node, "childclass", &child_class)) {
      child_class = parent_class;
    }

    bool compute_inertia;
    SpatialInertia<double> M_BBo_B(0, {0, 0, 0}, {0, 0, 0});
    XMLElement* inertial_node = node->FirstChildElement("inertial");
    if (inertial_node && (!inertia_from_geom_ || !*inertia_from_geom_)) {
      // Then we have a node and inertial_from_geom is "auto" or "false".
      M_BBo_B = ParseInertial(inertial_node);
      compute_inertia = false;
    } else if (!inertial_node && inertia_from_geom_ && !inertia_from_geom_) {
      // We don't have a node and inertial_from_geom is "false".
      // https://mujoco.readthedocs.io/en/latest/XMLreference.html#compiler
      // says we should throw.
      throw std::runtime_error(fmt::format(
          "{} has no inertial tag and inertiafromgeom=false. You must specify "
          "an inertia.",
          body_name));
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
      if (compute_inertia) {
        M_BBo_B += geom.M_GBo_B;
      }
      geometries.push_back(std::move(geom));
    }

    // Add a rigid body to model each link.
    const RigidBody<double>& body =
        plant_->AddRigidBody(body_name, model_instance_, M_BBo_B);

    if (plant_->geometry_source_is_registered()) {
      for (auto& geom : geometries) {
        plant_->RegisterVisualGeometry(body, geom.X_BG, *geom.shape, geom.name,
                                       geom.rgba);
        plant_->RegisterCollisionGeometry(body, geom.X_BG, *geom.shape,
                                          geom.name, geom.friction);
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
              SpatialInertia<double>(0, {0, 0, 0}, {0, 0, 0}));
          ParseJoint(joint_node, *last_body, dummy_body, X_WP,
                     RigidTransformd());
          last_body = &dummy_body;
        } else {
          ParseJoint(joint_node, *last_body, body, X_WB, X_PB);
        }

        std::string type;
        ParseStringAttribute(joint_node, "type", &type);
        if (type == "free") {
          if (dummy_bodies > 0) {
            throw std::logic_error(
                fmt::format("No other joints can be defined in the body {} if "
                            "a free joint is defined.",
                            body_name));
          }
          break;  // No other joints are allowed if the joint is "free".
        }
      }
    } else {  // no "joint" element.
      if (XMLElement* freejoint_node = node->FirstChildElement("freejoint")) {
        WarnUnsupportedElement(*freejoint_node, "name");
        WarnUnsupportedElement(*freejoint_node, "group");
        plant_->SetDefaultFreeBodyPose(body, X_WB);
      } else {
        plant_->WeldFrames(parent.body_frame(), body.body_frame(), X_PB);
      }
    }

    WarnUnsupportedAttribute(*node, "mocap");
    WarnUnsupportedAttribute(*node, "user");

    WarnUnsupportedElement(*node, "include");
    WarnUnsupportedElement(*node, "site");
    WarnUnsupportedElement(*node, "camera");
    WarnUnsupportedElement(*node, "light");
    WarnUnsupportedElement(*node, "composite");

    // Parses child body elements.
    for (XMLElement* link_node = node->FirstChildElement("body"); link_node;
         link_node = link_node->NextSiblingElement("body")) {
      ParseBody(link_node, body, X_WB, child_class);
    }
  }

  void ParseWorldBody(XMLElement* node) {
    if (plant_->geometry_source_is_registered()) {
      std::vector<MujocoGeometry> geometries;
      for (XMLElement* link_node = node->FirstChildElement("geom"); link_node;
           link_node = link_node->NextSiblingElement("geom")) {
        auto geom = ParseGeometry(link_node, geometries.size(), false);
        if (!geom.shape) continue;
        plant_->RegisterVisualGeometry(plant_->world_body(), geom.X_BG,
                                       *geom.shape, geom.name, geom.rgba);
        plant_->RegisterCollisionGeometry(plant_->world_body(), geom.X_BG,
                                          *geom.shape, geom.name,
                                          geom.friction);
      }
    }

    WarnUnsupportedElement(*node, "include");
    WarnUnsupportedElement(*node, "site");
    WarnUnsupportedElement(*node, "camera");
    WarnUnsupportedElement(*node, "light");
    WarnUnsupportedElement(*node, "composite");

    // Parses child body elements.
    for (XMLElement* link_node = node->FirstChildElement("body"); link_node;
         link_node = link_node->NextSiblingElement("body")) {
      ParseBody(link_node, plant_->world_body(), RigidTransformd());
    }
  }

  void ParseDefault(XMLElement* node, const std::string& parent_default = "") {
    std::string class_name;
    if (!ParseStringAttribute(node, "class", &class_name)) {
      if (parent_default.empty()) {
        class_name = "main";
      } else {
        throw std::logic_error(
            "The `class` attribute is required for all `default` elements "
            "except at the top-level");
      }
    }

    // Parse default geometries.
    for (XMLElement* geom_node = node->FirstChildElement("geom"); geom_node;
         geom_node = geom_node->NextSiblingElement("geom")) {
      default_geometry_[class_name] = geom_node;
      if (!parent_default.empty() &&
          default_geometry_.count(parent_default) > 0) {
        ApplyDefaultAttributes(*default_geometry_.at(parent_default),
                               geom_node);
      }
    }

    // Parse child defaults.
    for (XMLElement* default_node = node->FirstChildElement("default");
         default_node;
         default_node = default_node->NextSiblingElement("default")) {
      ParseDefault(default_node, class_name);
    }

    WarnUnsupportedElement(*node, "include");
    WarnUnsupportedElement(*node, "mesh");
    WarnUnsupportedElement(*node, "material");
    WarnUnsupportedElement(*node, "site");
    WarnUnsupportedElement(*node, "camera");
    WarnUnsupportedElement(*node, "light");
    WarnUnsupportedElement(*node, "pair");
    WarnUnsupportedElement(*node, "equality");
    WarnUnsupportedElement(*node, "tendon");
    WarnUnsupportedElement(*node, "general");
    WarnUnsupportedElement(*node, "motor");
    WarnUnsupportedElement(*node, "position");
    WarnUnsupportedElement(*node, "velocity");
    WarnUnsupportedElement(*node, "cylinder");
    WarnUnsupportedElement(*node, "muscle");
  }

  void ParseAsset(XMLElement* node) {
    for (XMLElement* material_node = node->FirstChildElement("material");
         material_node;
         material_node = material_node->NextSiblingElement("material")) {
      std::string name;
      if (!ParseStringAttribute(material_node, "name", &name)) {
        throw std::runtime_error(
            "ERROR: Material elements must have a name attribute.");
      }
      material_[name] = material_node;
    }

    for (XMLElement* mesh_node = node->FirstChildElement("mesh");
         mesh_node;
         mesh_node = mesh_node->NextSiblingElement("mesh")) {
      WarnUnsupportedAttribute(*mesh_node, "class");
      WarnUnsupportedAttribute(*mesh_node, "smoothnormal");
      WarnUnsupportedAttribute(*mesh_node, "vertex");
      // Note: "normal" and "face" are not supported either, but that lack of
      // support is implied by us not supporting "vertex".
      WarnUnsupportedAttribute(*mesh_node, "refpos");
      WarnUnsupportedAttribute(*mesh_node, "refquat");

      std::string file;
      if (ParseStringAttribute(mesh_node, "file", &file)) {
        std::string name{file};
        ParseStringAttribute(mesh_node, "name", &name);

        Vector3d scale{1, 1, 1};
        if (ParseVectorAttribute(mesh_node, "scale", &scale)) {
          if (scale[0] != scale[1] || scale[1] != scale[2]) {
            throw std::runtime_error(
                fmt::format("mesh {} was defined with a non-uniform scale; but "
                            "Drake currently only supports uniform scaling",
                            name));
          }
        }

        std::filesystem::path filename(file);

        /* Adapted from the mujoco docs: The full path to a file is determined
        as follows. If the strippath compiler option is “true”, all path
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
        filename = std::filesystem::canonical(filename);
        std::filesystem::path original_filename = filename;

        // TODO(russt): Support .vtk files.

        // Replace the extension with obj, unless tolower(extension) == "obj".
        std::string extension = filename.extension();
        std::transform(extension.begin(), extension.end(), extension.begin(),
                        [](unsigned char c) { return std::tolower(c); });
        if (extension != ".obj") {
          filename.replace_extension("obj");
        }

        if (std::filesystem::exists(filename)) {
          mesh_[name] =
              std::make_unique<geometry::Mesh>(filename, scale[0]);
          surface_mesh_.emplace(std::pair(
              name,
              geometry::ReadObjToTriangleSurfaceMesh(filename, scale[0])));
        } else if (std::filesystem::exists(original_filename)) {
          log()->warn(
              "Drake's MuJoCo parser currently only supports mesh files in "
              ".obj format. The meshfile {} was requested; Drake attempted to "
              "load {}, but that file does not exist.",
              original_filename, filename);
          std::string original_extension = original_filename.extension();
          std::transform(original_extension.begin(), original_extension.end(),
                         original_extension.begin(),
                         [](unsigned char c) { return std::tolower(c); });
          if (original_extension == ".stl") {
            log()->warn(
                "If you have built Drake from source, running\n\n bazel run "
                "//manipulation/utils/stl2obj -- {} {}\n\nonce will "
                "resolve this.",
                original_filename, filename);
          }
        } else {
          log()->warn(
              "The mesh asset {} could not be found, nor could its .obj "
              "replacement {}.",
              original_filename, filename);
        }
      } else {
        std::string name{};
        ParseStringAttribute(mesh_node, "name", &name);
        log()->warn(
            "The mesh asset named {} did not specify a 'file' attribute and so "
            "will be ignored.",
            name);
      }
    }

    WarnUnsupportedElement(*node, "texture");
    WarnUnsupportedElement(*node, "hfield");
    WarnUnsupportedElement(*node, "skin");
  }

  void ParseOption(XMLElement* node) {
    WarnUnsupportedAttribute(*node, "timestep");
    WarnUnsupportedAttribute(*node, "apirate");
    WarnUnsupportedAttribute(*node, "impratio");

    Vector3d gravity;
    if (ParseVectorAttribute(node, "gravity", &gravity)) {
      // Note: This changes gravity for the entire plant (including models that
      // existed before this parser).
      plant_->mutable_gravity_field().set_gravity_vector(gravity);
    }

    WarnUnsupportedAttribute(*node, "wind");
    WarnUnsupportedAttribute(*node, "magnetic");
    WarnUnsupportedAttribute(*node, "density");
    WarnUnsupportedAttribute(*node, "viscosity");
    WarnUnsupportedAttribute(*node, "o_margin");
    WarnUnsupportedAttribute(*node, "o_solref");
    WarnUnsupportedAttribute(*node, "o_solimp");
    WarnUnsupportedAttribute(*node, "integrator");
    WarnUnsupportedAttribute(*node, "collision");
    WarnUnsupportedAttribute(*node, "cone");
    WarnUnsupportedAttribute(*node, "jacobian");
    WarnUnsupportedAttribute(*node, "solver");
    WarnUnsupportedAttribute(*node, "iterations");
    WarnUnsupportedAttribute(*node, "tolerance");
    WarnUnsupportedAttribute(*node, "noslip_iterations");
    WarnUnsupportedAttribute(*node, "noslip_tolerance");
    WarnUnsupportedAttribute(*node, "mpr_iterations");
    WarnUnsupportedAttribute(*node, "mpr_tolerance");
  }

  void ParseCompiler(XMLElement* node) {
    WarnUnsupportedAttribute(*node, "boundmass");
    WarnUnsupportedAttribute(*node, "boundinertia");
    WarnUnsupportedAttribute(*node, "settotalmass");
    WarnUnsupportedAttribute(*node, "balanceinertia");
    WarnUnsupportedAttribute(*node, "strippath");
    std::string coordinate;
    if (ParseStringAttribute(node, "coordinate", &coordinate)) {
      if (coordinate != "local") {
        throw std::logic_error(fmt::format(
            "Compiler attribute coordinate={} is not supported yet.",
            coordinate));
      }
    }
    std::string angle;
    if (ParseStringAttribute(node, "angle", &angle)) {
      if (angle == "degree") {
        angle_ = kDegree;
      } else if (angle == "radian") {
        angle_ = kRadian;
      } else {
        drake::log()->warn(
            "Unknown value {} for the compiler `angle` attribute will be "
            "ignored.  The existing value of `angle={}` will remain unchanged.",
            angle, (angle_ == kDegree) ? "degree" : "radian");
      }
    }

    std::string meshdir;
    if (ParseStringAttribute(node, "meshdir", &meshdir)) {
      meshdir_ = meshdir;
    }

    WarnUnsupportedAttribute(*node, "fitaabb");
    WarnUnsupportedAttribute(*node, "eulerseq");
    WarnUnsupportedAttribute(*node, "texturedir");
    WarnUnsupportedAttribute(*node, "discardvisual");
    WarnUnsupportedAttribute(*node, "convexhull");
    // Note: we intentionally (silently) ignore "usethread" attribute.
    WarnUnsupportedAttribute(*node, "fusestatic");

    bool flag;
    switch (node->QueryBoolAttribute("inertiafromgeom", &flag)) {
      case tinyxml2::XML_SUCCESS:
        inertia_from_geom_ = flag;
        break;
      case tinyxml2::XML_WRONG_ATTRIBUTE_TYPE:
        if (std::string(node->Attribute("inertiafromgeom")) == "auto") {
          break;
        }
        drake::log()->warn(
            "The attribute 'inertiafromgeom' found in a '{}' tag cannot be "
            "interpreted as a boolean nor 'auto' and will be ignored.",
            node->Name());
        break;
      default:
        // Ok. No attribute to set.
        break;
    }
    WarnUnsupportedAttribute(*node, "inertiagrouprange");
  }

  // Assets without an absolute path are referenced relative to the "main MJCF
  // model file" path, `main_mjcf_path`.
  ModelInstanceIndex Parse(const std::string& model_name_in,
                           const std::optional<std::string>& parent_model_name,
                           XMLDocument* xml_doc,
                           const std::filesystem::path& main_mjcf_path) {
    main_mjcf_path_ = main_mjcf_path;

    XMLElement* node = xml_doc->FirstChildElement("mujoco");
    if (!node) {
      throw std::runtime_error("ERROR: XML does not contain a mujoco tag.");
    }

    std::string model_name = model_name_in;
    if (model_name.empty() &&
        !ParseStringAttribute(node, "model", &model_name)) {
      throw std::runtime_error(
          "ERROR: Your robot must have a name attribute or a model name "
          "must be specified.");
    }
    model_name =
        parsing::PrefixName(parent_model_name.value_or(""), model_name);

    model_instance_ = plant_->AddModelInstance(model_name);

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

    // Parses the assets.
    for (XMLElement* asset_node = node->FirstChildElement("asset");
         asset_node;
         asset_node = asset_node->NextSiblingElement("asset")) {
      ParseAsset(asset_node);
    }

    // Parse the defaults.
    for (XMLElement* default_node = node->FirstChildElement("default");
         default_node;
         default_node = default_node->NextSiblingElement("default")) {
      ParseDefault(default_node);
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

    WarnUnsupportedElement(*node, "include");
    WarnUnsupportedElement(*node, "size");
    WarnUnsupportedElement(*node, "visual");
    WarnUnsupportedElement(*node, "statistic");
    WarnUnsupportedElement(*node, "custom");
    WarnUnsupportedElement(*node, "contact");
    WarnUnsupportedElement(*node, "equality");
    WarnUnsupportedElement(*node, "tendon");
    WarnUnsupportedElement(*node, "sensor");
    WarnUnsupportedElement(*node, "keyframe");

    return model_instance_;
  }

 private:
  MultibodyPlant<double>* plant_;
  ModelInstanceIndex model_instance_{};
  std::filesystem::path main_mjcf_path_{};
  enum Angle { kRadian, kDegree };
  Angle angle_{kDegree};
  std::map<std::string, XMLElement*> default_geometry_{};
  std::optional<bool> inertia_from_geom_{};  // !has_value() => "auto".
  std::map<std::string, XMLElement*> material_{};
  std::optional<std::filesystem::path> meshdir_{};
  std::map<std::string, std::unique_ptr<geometry::Mesh>> mesh_{};
  std::map<std::string, geometry::TriangleSurfaceMesh<double>> surface_mesh_{};
};

}  // namespace

ModelInstanceIndex AddModelFromMujocoXml(
    const DataSource& data_source, const std::string& model_name_in,
    const std::optional<std::string>& parent_model_name,
    MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(!plant->is_finalized());

  XMLDocument xml_doc;
  std::filesystem::path path{};
  if (data_source.IsFilename()) {
    xml_doc.LoadFile(data_source.filename().c_str());
    if (xml_doc.ErrorID()) {
      throw std::runtime_error(fmt::format("Failed to parse XML file {}:\n{}",
                                           data_source.filename(),
                                           xml_doc.ErrorName()));
    }
    path = data_source.filename();
    path.remove_filename();
  } else {
    xml_doc.Parse(data_source.contents().c_str());
    if (xml_doc.ErrorID()) {
      throw std::runtime_error(
          fmt::format("Failed to parse XML string: {}", xml_doc.ErrorName()));
    }
    path = std::filesystem::current_path();
  }

  MujocoParser parser(plant);
  return parser.Parse(model_name_in, parent_model_name, &xml_doc, path);
}

MujocoParserWrapper::MujocoParserWrapper() {}

MujocoParserWrapper::~MujocoParserWrapper() {}

std::optional<ModelInstanceIndex> MujocoParserWrapper::AddModel(
    const DataSource& data_source, const std::string& model_name,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  return AddModelFromMujocoXml(data_source, model_name, parent_model_name,
                               workspace.plant);
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
