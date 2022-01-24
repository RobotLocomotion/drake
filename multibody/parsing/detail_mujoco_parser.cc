#include "drake/multibody/parsing/detail_mujoco_parser.h"

#include <memory>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <tinyxml2.h>

#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/detail_tinyxml.h"
#include "drake/multibody/parsing/scoped_names.h"

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
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

namespace {

// For tags that exist in the MuJoCo XML schema, but are not (yet) supported
// here, we print a warning to the console.  This method checks the node for
// the child element, and prints the warning if any occurrences exist.
void WarnUnsupportedElement(const XMLElement& node, const std::string& tag) {
  if (node.FirstChildElement(tag.c_str())) {
    drake::log()->warn(
        "The tag '{}' found in MuJoCo XML is currently unsupported and will be "
        "ignored.",
        tag);
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

class MujocoParser {
 public:
  explicit MujocoParser(MultibodyPlant<double>* plant) : plant_{plant} {}

  RigidTransformd ParseTransform(XMLElement* node) {
    Vector3d pos{0, 0, 0};  // Default position is zero.
    ParseVectorAttribute(node, "pos", &pos);

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

    return RigidTransformd(pos);
  }

  struct MujocoGeometry {
    RigidTransformd X_BG{};
    std::string name{};
    std::unique_ptr<geometry::Shape> shape{};
    Vector4d rgba{.5, .5, .5, 1};
    CoulombFriction<double> friction{1.0, 1.0};
  };

  MujocoGeometry ParseGeometry(XMLElement* node, int num_geom) {
    MujocoGeometry geom;
    geom.X_BG = ParseTransform(node);

    if (!ParseStringAttribute(node, "name", &geom.name)) {
      // Use "geom#" as the default body name.
      geom.name = fmt::format("geom{}", num_geom);
    }

    WarnUnsupportedAttribute(*node, "class");

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
    } else if (type == "capsule") {
      if (has_fromto) {
        if (size.size() < 1) {
          throw std::logic_error(
              "The size attribute for capsule geom using fromto must have at "
              "least one element.");
        }
        double length = (fromto.head<3>() - fromto.tail<3>()).norm();
        geom.shape = std::make_unique<geometry::Capsule>(size[0], length);
      } else {
        if (size.size() < 2) {
          throw std::logic_error(
              "The size attribute for capsule geom must have at least two "
              "elements.");
        }
        geom.shape = std::make_unique<geometry::Capsule>(size[0], 2 * size[1]);
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
      } else {
        if (size.size() < 2) {
          throw std::logic_error(
              "The size attribute for cylinder geom must have at least two "
              "elements.");
        }
        geom.shape = std::make_unique<geometry::Cylinder>(size[0], 2 * size[1]);
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
              "The size attribute for box geom must have at least two "
              "elements.");
        }
        geom.shape = std::make_unique<geometry::Box>(
            size[0] * 2.0, size[1] * 2.0, size[2] * 2.0);
      }
    } else if (type == "mesh" || type == "hfield") {
      drake::log()->warn(
          "The geom type '{}' is currently unsupported and will be ignored.",
          type);
      return geom;
    } else {
      throw std::runtime_error(fmt::format("Unrecognized geom type {}", type));
    }

    WarnUnsupportedAttribute(*node, "contype");
    WarnUnsupportedAttribute(*node, "conaffinity");
    WarnUnsupportedAttribute(*node, "condim");
    WarnUnsupportedAttribute(*node, "group");
    WarnUnsupportedAttribute(*node, "priority");
    WarnUnsupportedAttribute(*node, "material");

    ParseVectorAttribute(node, "rgba", &geom.rgba);

    Vector3d friction{1, 0.005, 0.0001};
    if (ParseVectorAttribute(node, "friction", &friction)) {
      // MuJoCo's friction specification is [sliding, torsional, rolling].  We
      // set the static and dynamic friction to `sliding`, and do not support
      // the other parameters.
      geom.friction = CoulombFriction(friction[0], friction[0]);
      if (friction[1] != 0.0 || friction[2] != 0.0) {
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

    WarnUnsupportedAttribute(*node, "mass");
    WarnUnsupportedAttribute(*node, "density");

    return geom;
  }

  void ParseWorldBody(XMLElement* node) {
    if (plant_->geometry_source_is_registered()) {
      std::vector<MujocoGeometry> geometries;
      for (XMLElement* link_node = node->FirstChildElement("geom"); link_node;
          link_node = link_node->NextSiblingElement("geom")) {
        auto geom = ParseGeometry(link_node, geometries.size());
        if (!geom.shape) continue;
        plant_->RegisterVisualGeometry(plant_->world_body(), geom.X_BG,
                                       *geom.shape, geom.name, geom.rgba);
        plant_->RegisterCollisionGeometry(plant_->world_body(), geom.X_BG,
                                          *geom.shape, geom.name,
                                          geom.friction);
      }
    }

    WarnUnsupportedElement(*node, "body");
    WarnUnsupportedElement(*node, "site");
    WarnUnsupportedElement(*node, "camera");
    WarnUnsupportedElement(*node, "light");
    WarnUnsupportedElement(*node, "composite");
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
    WarnUnsupportedAttribute(*node, "coordinate");
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
    WarnUnsupportedAttribute(*node, "fitaabb");
    WarnUnsupportedAttribute(*node, "eulerseq");
    WarnUnsupportedAttribute(*node, "meshdir");
    WarnUnsupportedAttribute(*node, "texturedir");
    WarnUnsupportedAttribute(*node, "discardvisual");
    WarnUnsupportedAttribute(*node, "convexhull");
    // Note: we intentionally (silently) ignore "usethread" attribute.
    WarnUnsupportedAttribute(*node, "fusestatic");
    WarnUnsupportedAttribute(*node, "inertiafromgeom");
    WarnUnsupportedAttribute(*node, "inertiagrouprange");
  }

  ModelInstanceIndex Parse(const std::string& model_name_in,
                           const std::optional<std::string>& parent_model_name,
                           XMLDocument* xml_doc) {
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
         option_node;
         option_node = option_node->NextSiblingElement("option")) {
      ParseOption(option_node);
    }

    // Parses the model's world link elements.
    for (XMLElement* link_node = node->FirstChildElement("worldbody");
         link_node; link_node = link_node->NextSiblingElement("worldbody")) {
      ParseWorldBody(link_node);
    }

    // These will all be (at least partially) supported via #16369.
    WarnUnsupportedElement(*node, "body");
    WarnUnsupportedElement(*node, "actuator");

    // These will not be supported initially.
    WarnUnsupportedElement(*node, "size");
    WarnUnsupportedElement(*node, "visual");
    WarnUnsupportedElement(*node, "statistic");
    WarnUnsupportedElement(*node, "default");
    WarnUnsupportedElement(*node, "custom");
    WarnUnsupportedElement(*node, "asset");
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
  enum Angle { kRadian, kDegree };
  Angle angle_{kDegree};
};

}  // namespace

ModelInstanceIndex AddModelFromMujocoXml(
    const DataSource& data_source, const std::string& model_name_in,
    const std::optional<std::string>& parent_model_name,
    MultibodyPlant<double>* plant, geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(!plant->is_finalized());
  data_source.DemandExactlyOne();

  XMLDocument xml_doc;
  if (data_source.file_name) {
    xml_doc.LoadFile(data_source.file_name->c_str());
    if (xml_doc.ErrorID()) {
      throw std::runtime_error(fmt::format("Failed to parse XML file {}:\n{}",
                                           *data_source.file_name,
                                           xml_doc.ErrorName()));
    }
  } else {
    DRAKE_DEMAND(data_source.file_contents != nullptr);
    xml_doc.Parse(data_source.file_contents->c_str());
    if (xml_doc.ErrorID()) {
      throw std::runtime_error(
          fmt::format("Failed to parse XML string: {}", xml_doc.ErrorName()));
    }
  }

  if (scene_graph != nullptr && !plant->geometry_source_is_registered()) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
  }

  MujocoParser parser(plant);
  return parser.Parse(model_name_in, parent_model_name, &xml_doc);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
