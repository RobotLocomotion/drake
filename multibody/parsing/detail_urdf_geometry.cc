#include "drake/multibody/parsing/detail_urdf_geometry.h"

#include <iomanip>
#include <memory>
#include <ostream>
#include <sstream>
#include <stdexcept>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/parsing/detail_tinyxml.h"

namespace drake {
namespace multibody {
namespace internal {

using Eigen::Vector3d;
using Eigen::Vector4d;
using math::RigidTransformd;
using tinyxml2::XMLElement;

namespace {

// Adds a material to the supplied @materials map. Currently, only simple
// colors are supported.
//
// TODO(sammy-tri) Add support for texture-based materials. See:
// https://github.com/RobotLocomotion/drake/issues/2588.
//
// @param[in] material_name A human-understandable name of the material.
//
// @param[in] color_rgba The red-green-blue-alpha color values of the material.
// The range of values is [0, 1].
//
// @param[in] abort_if_name_clash If true, this method will abort if
// @p material_name is already in @p materials regardless of whether the RGBA
// values are the same. If false, this method will abort if
// @p material_name is already in @p materials and the infinity norm of the
// difference is greater than 1e-10.
//
// @param[out] materials A pointer to the map in which to store the material.
// This cannot be nullptr.
void AddMaterialToMaterialMap(const std::string& material_name,
                              const Vector4d& color_rgba,
                              bool abort_if_name_clash,
                              MaterialMap* materials) {
  DRAKE_DEMAND(materials);

  // Determines if the material is already in the map.
  auto material_iter = materials->find(material_name);
  if (material_iter != materials->end()) {
    // The material is already in the map. Checks whether the old material is
    // the same as the new material.  The range of values in the RGBA vectors
    // is [0, 1].
    const Vector4d& existing_color = material_iter->second;
    if (abort_if_name_clash || (color_rgba != existing_color)) {
      // The materials map already has the material_name key but the color
      // associated with it is different.
      std::stringstream error_buff;
      error_buff << "Material \"" + material_name + "\" was previously "
                 << "defined.\n"
                 << "  - existing RGBA values: " << existing_color.transpose()
                 << std::endl
                 << "  - new RGBA values: " << color_rgba.transpose()
                 << std::endl;
      throw std::runtime_error(error_buff.str());
    }
  } else {
    // Adds the new color to the materials map.
    (*materials)[material_name] = color_rgba;
  }
}

}  // namespace

void ParseMaterial(const XMLElement* node, MaterialMap* materials) {
  if (std::string(node->Name()) != "material") {
    throw std::runtime_error(std::string("Expected material element, got ") +
                             node->Name());
  }

  std::string name;
  ParseStringAttribute(node, "name", &name);
  if (name.empty()) {
    throw std::runtime_error("Material tag is missing a name.");
  }

  Vector4d rgba = Vector4d::Zero();  // Defaults to black.

  const XMLElement* color_node = node->FirstChildElement("color");

  if (color_node) {
    if (!ParseVectorAttribute(color_node, "rgba", &rgba)) {
      throw std::runtime_error("Color tag is missing rgba attribute.");
    }
    AddMaterialToMaterialMap(name, rgba, true /* abort_if_name_clash */,
                             materials);
  } else {
    // If no color was specified and the material is not in the materials map,
    // check if the material is texture-based. If it is, print a warning, use
    // default color (black), and then return.
    //
    // Otherwise, throw an exception.
    //
    // TODO(sammy-tri): Update this logic once texture-based materials are
    // supported. See: https://github.com/RobotLocomotion/drake/issues/2588.
    if (materials->find(name) == materials->end()) {
      const XMLElement* texture_node = node->FirstChildElement("texture");

      if (texture_node) {
        std::stringstream error_buff;
        error_buff << "WARNING: Material \"" << name
                   << "\" is a texture. Textures are currently "
                   << "not supported. For more information, see: "
                   << "https://github.com/RobotLocomotion/drake/issues/2588. "
                   << "Defaulting to use the black color for this material.";
        drake::log()->warn(error_buff.str());

        AddMaterialToMaterialMap(name, rgba, true /* abort_if_name_clash */,
                                 materials);
      } else {
        throw std::runtime_error("Material\"" + name +
                                 "\" not previously defined. Therefore "
                                 "a color must be specified.");
      }
    }
  }
}

namespace {

std::unique_ptr<geometry::Shape> ParseBox(const XMLElement* shape_node) {
  Eigen::Vector3d size = Eigen::Vector3d::Zero();
  if (!ParseVectorAttribute(shape_node, "size", &size)) {
    throw std::runtime_error("Missing box attribute: size");
  }
  return std::make_unique<geometry::Box>(size(0), size(1), size(2));
}

std::unique_ptr<geometry::Shape> ParseSphere(const XMLElement* shape_node) {
  double r = 0;
  if (!ParseScalarAttribute(shape_node, "radius", &r)) {
    throw std::runtime_error("Missing sphere attribute: radius");
  }

  // TODO(sammy-tri) Do we need to enforce a minimum radius here?  The old
  // RBT-based parser did.  See
  // https://github.com/RobotLocomotion/drake/issues/4555
  return std::make_unique<geometry::Sphere>(r);
}

std::unique_ptr<geometry::Shape> ParseCylinder(const XMLElement* shape_node) {
  double r = 0;
  if (!ParseScalarAttribute(shape_node, "radius", &r)) {
    throw std::runtime_error("Missing cylinder attribute: radius");
  }

  double l = 0;
  if (!ParseScalarAttribute(shape_node, "length", &l)) {
    throw std::runtime_error("Missing cylinder attribute: length");
  }
  return std::make_unique<geometry::Cylinder>(r, l);
}

std::unique_ptr<geometry::Shape> ParseCapsule(const XMLElement* shape_node) {
  double r = 0;
  if (!ParseScalarAttribute(shape_node, "radius", &r)) {
    throw std::runtime_error("Missing capsule attribute: radius");
  }

  double l = 0;
  if (!ParseScalarAttribute(shape_node, "length", &l)) {
    throw std::runtime_error("Missing capsule attribute: length");
  }
  return std::make_unique<geometry::Capsule>(r, l);
}

std::unique_ptr<geometry::Shape> ParseMesh(const XMLElement* shape_node,
                                           const PackageMap& package_map,
                                           const std::string& root_dir) {
  std::string filename;
  if (!ParseStringAttribute(shape_node, "filename", &filename)) {
    throw std::runtime_error("Mesh element has no filename tag");
  }

  const std::string resolved_filename =
      ResolveUri(filename, package_map, root_dir);
  if (resolved_filename.empty()) {
    throw std::runtime_error(
        "Mesh file name could not be resolved from the provided uri \"" +
        filename + "\".");
  }

  double scale = 1.0;
  // Obtains the scale of the mesh if it exists.
  if (shape_node->Attribute("scale") != nullptr) {
    Vector3d scale_vector;
    ParseThreeVectorAttribute(shape_node, "scale", &scale_vector);
    // geometry::Mesh only supports isotropic scaling and therefore we
    // enforce it.
    if (!(scale_vector(0) == scale_vector(1) &&
          scale_vector(0) == scale_vector(2))) {
      throw std::runtime_error(
          "Drake meshes only support isotropic scaling. Therefore all "
          "three scaling factors must be exactly equal.");
    }
    scale = scale_vector(0);
  }

  if (shape_node->FirstChildElement("drake:declare_convex")) {
    return std::make_unique<geometry::Convex>(resolved_filename, scale);
  } else {
    return std::make_unique<geometry::Mesh>(resolved_filename, scale);
  }
}

std::unique_ptr<geometry::Shape> ParseGeometry(const XMLElement* node,
                                               const PackageMap& package_map,
                                               const std::string& root_dir) {
  if (node->FirstChildElement("box")) {
    return ParseBox(node->FirstChildElement("box"));
  }
  if (node->FirstChildElement("sphere")) {
    return ParseSphere(node->FirstChildElement("sphere"));
  }
  if (node->FirstChildElement("cylinder")) {
    return ParseCylinder(node->FirstChildElement("cylinder"));
  }
  if (node->FirstChildElement("capsule")) {
    return ParseCapsule(node->FirstChildElement("capsule"));
  }
  if (node->FirstChildElement("mesh")) {
    return ParseMesh(node->FirstChildElement("mesh"), package_map, root_dir);
  }

  throw std::runtime_error(
      "Warning: geometry element "
      "has an unknown type and will be ignored.");
}

std::string MakeGeometryName(const std::string& basename,
                             const XMLElement* node) {
  using std::hex;
  using std::setfill;
  using std::setw;

  // Append the address spelled like "@0123456789abcdef".
  intptr_t address = reinterpret_cast<intptr_t>(node);
  std::ostringstream result;
  result << basename << '@' << setfill('0') << setw(16) << hex << address;
  return result.str();
}

}  // namespace

// Parses a "visual" element in @p node.
geometry::GeometryInstance ParseVisual(const std::string& parent_element_name,
                                       const PackageMap& package_map,
                                       const std::string& root_dir,
                                       const XMLElement* node,
                                       MaterialMap* materials) {
  if (std::string(node->Name()) != "visual") {
    throw std::runtime_error("In link " + parent_element_name +
                             " expected visual element, got " + node->Name());
  }

  // Ensures there is a geometry child element. Since this is a required
  // element, throws an exception if a geometry element does not exist.
  const XMLElement* geometry_node = node->FirstChildElement("geometry");
  if (!geometry_node) {
    throw std::runtime_error("Link " + parent_element_name +
                             " has a visual element without geometry.");
  }

  // Obtains the reference frame of the visualization relative to the reference
  // frame of the rigid body that is being visualized. It defaults to identity
  // if no transform is specified.
  RigidTransformd T_element_to_link;
  const XMLElement* origin = node->FirstChildElement("origin");
  if (origin) {
    T_element_to_link = OriginAttributesToTransform(origin);
  }

  // Parses the geometry specifications of the visualization.
  std::unique_ptr<geometry::Shape> shape =
      ParseGeometry(geometry_node, package_map, root_dir);

  // Parses the material specification of the visualization. Note that we cannot
  // reuse the logic within ParseMaterial() here because the context is
  // different. Whereas ParseMaterial() parses material specifications that
  // are children elements of the "robot" element, the material specification
  // being parsed here are children of a "visual" element. One key difference is
  // the XML here may not specify a "name" attribute. Because of this difference
  // in context, we need specialized logic here to determine the material
  // visualization of a link.

  // The empty set relies on consumer defaults.
  geometry::IllustrationProperties properties;

  const XMLElement* material_node = node->FirstChildElement("material");
  if (material_node) {
    // Checks and remembers whether a "color" child element exists. If so,
    // parses the color value.
    bool color_specified = false;
    Vector4d rgba;
    const XMLElement* color_node = material_node->FirstChildElement("color");
    if (color_node) {
      if (!ParseVectorAttribute(color_node, "rgba", &rgba)) {
        throw std::runtime_error("Failed to parse color of material for link " +
                                 parent_element_name);
      }
      color_specified = true;
    }

    // Checks and remembers whether a "name" attribute exists. If so, parses the
    // name value.
    std::string material_name;
    bool name_specified = false;
    if (ParseStringAttribute(material_node, "name", &material_name) &&
        !material_name.empty()) {
      name_specified = true;
    }

    // Adds the material to the materials map if both the name and color are
    // specified. This is so that link elements that reside later in the URDF
    // can reference this material in their visualization elements. Note that
    // this capability is not specified by the official URDF specification (see:
    // http://wiki.ros.org/urdf/XML/link), but is needed by certain URDFs
    // released by companies and organizations like Robotiq and ROS Industrial
    // (for example, see this URDF by Robotiq: http://bit.ly/28P0pmo).
    if (color_specified && name_specified) {
      // The `abort_if_name_clash` parameter is passed a value of `false` to
      // allow the same material to be defined across multiple links as long as
      // they correspond to the same RGBA value. This can happen, for example,
      // in URDFs that are automatically generated using `xacro` since `xacro`
      // may produce a URDF from multiple `.xacro` files. Through testing, we
      // determined that the Gazebo simulator supports loading URDFs containing
      // duplicate material specifications as long as the duplicates are
      // distributed across multiple `<link>` elements and are not at the
      // `<robot>` level.
      AddMaterialToMaterialMap(material_name, rgba,
                               false /* abort_if_name_clash */, materials);
    }

    // If the color is specified as a child element of the current material
    // node, use that color. It takes precedence over any material saved in
    // the material map.
    if (color_specified) {
      properties = geometry::MakePhongIllustrationProperties(rgba);
    } else if (name_specified) {
      // No color specified. Checks if the material is already in the
      // materials map.

      auto material_iter = materials->find(material_name);
      if (material_iter != materials->end()) {
        // The material is in the map. Sets the material of the visual
        // element based on the value in the map.
        properties =
            geometry::MakePhongIllustrationProperties(material_iter->second);
      }
    }
  }

  std::string geometry_name;
  if (!ParseStringAttribute(node, "name", &geometry_name)) {
    geometry_name = MakeGeometryName(parent_element_name + "_Visual", node);
  }

  auto instance = geometry::GeometryInstance(T_element_to_link,
                                             std::move(shape), geometry_name);
  instance.set_illustration_properties(properties);
  return instance;
}

// This is the backwards-compatible fallback for defining friction; it reads
// the soon-to-be-deprecated <drake_compliance> tag for data. It throws errors
// for malformed values or returns a friction (either the valid friction
// defined in the tag or the default).
//
// It incidentally propagates some warnings about unused tags from the rigid
// body tree days.
CoulombFriction<double> ParseCoulombFrictionFromDrakeCompliance(
    const std::string& parent_element_name, const XMLElement* node) {
  const XMLElement* compliant_node =
      node->FirstChildElement("drake_compliance");
  if (compliant_node) {
    // TODO(SeanCurtis-TRI): Ultimately, we want to kill <drake_compliance>
    //  and these will go along with it. These values are only used in rigid
    //  body tree; with no real expectation we'll re-use them in MBP.
    if (compliant_node->FirstChildElement("youngs_modulus")) {
      drake::log()->warn("Ignoring youngs_modulus for link " +
                         parent_element_name);
    }

    if (compliant_node->FirstChildElement("dissipation")) {
      drake::log()->warn("Ignoring dissipation for link " +
                         parent_element_name);
    }

    double static_friction{-1};
    double dynamic_friction{-1};
    bool static_friction_present{false};
    bool dynamic_friction_present{false};

    const XMLElement* friction_node =
        compliant_node->FirstChildElement("static_friction");
    if (friction_node) {
      static_friction_present = true;
      if (friction_node->QueryDoubleText(&static_friction)) {
        throw std::runtime_error("Unable to parse static_friction for link " +
                                 parent_element_name);
      }
    }

    friction_node = compliant_node->FirstChildElement("dynamic_friction");
    if (friction_node) {
      dynamic_friction_present = true;
      if (friction_node->QueryDoubleText(&dynamic_friction)) {
        throw std::runtime_error("Unable to parse dynamic_friction for link " +
                                 parent_element_name);
      }
    }

    if (static_friction_present != dynamic_friction_present) {
      throw std::runtime_error(
          fmt::format("Link '{}': When specifying coefficient of friction, "
                      "both static and dynamic coefficients must be defined",
                      parent_element_name));
    }

    if (static_friction_present) {
      return CoulombFriction<double>(static_friction, dynamic_friction);
    }
  }
  return default_friction();
}

// Parses a "collision" element in @p node.
//
// @param[out] friction Coulomb friction for the associated geometry.
geometry::GeometryInstance ParseCollision(
    const std::string& parent_element_name, const PackageMap& package_map,
    const std::string& root_dir, const XMLElement* node) {
  if (std::string(node->Name()) != "collision") {
    throw std::runtime_error(
        fmt::format("In link '{}' expected collision element, got {}",
                    parent_element_name, node->Name()));
  }

  // Ensures there is a geometry child element. Since this is a required
  // element, throws an exception if a geometry element does not exist.
  const XMLElement* geometry_node = node->FirstChildElement("geometry");
  if (!geometry_node) {
    throw std::runtime_error(
        fmt::format("Link '{}' has a collision element without geometry",
                    parent_element_name));
  }

  // Obtains the reference frame of the visualization relative to the
  // reference frame of the rigid body that is being visualized. It defaults
  // to identity if no transform is specified.
  RigidTransformd T_element_to_link;
  const XMLElement* origin = node->FirstChildElement("origin");
  if (origin) {
    T_element_to_link = OriginAttributesToTransform(origin);
  }

  const char* attr = node->Attribute("group");
  if (attr) {
    drake::log()->warn("Ignoring collision group {} on link {}", attr,
                       parent_element_name);
  }

  std::unique_ptr<geometry::Shape> shape =
      ParseGeometry(geometry_node, package_map, root_dir);

  // Parse the properties from <drake:proximity_properties>.
  geometry::ProximityProperties props;
  const XMLElement* drake_element =
      node->FirstChildElement("drake:proximity_properties");
  if (drake_element) {
    auto read_double =
        [drake_element](const char* element_name) -> std::optional<double> {
      const XMLElement* value_node =
          drake_element->FirstChildElement(element_name);
      if (value_node != nullptr) {
        double value{};
        if (ParseScalarAttribute(value_node, "value", &value)) {
          return value;
        } else {
          throw std::runtime_error(
              fmt::format("Unable to read the 'value' attribute for the <{}> "
                          "tag on line {}",
                          element_name, value_node->GetLineNum()));
        }
      }
      return {};
    };

    const XMLElement* const rigid_element =
        drake_element->FirstChildElement("drake:rigid_hydroelastic");
    const XMLElement* const soft_element =
        drake_element->FirstChildElement("drake:soft_hydroelastic");
    if (rigid_element && soft_element) {
      throw std::runtime_error(fmt::format(
          "Collision geometry has defined mutually-exclusive tags "
          "<drake:rigid_hydroelastic> and <drake:soft_hydroelastic> on lines "
          "{} and {}, respectively. Only one can be provided.",
          rigid_element->GetLineNum(), soft_element->GetLineNum()));
    }

    props = ParseProximityProperties(read_double, rigid_element != nullptr,
        soft_element != nullptr);
  }

  // Now test to see how we should handle a potential <drake_compliance> tag.
  if (!props.HasProperty(geometry::internal::kMaterialGroup,
                         geometry::internal::kFriction)) {
    // We have no friction from <drake:proximity_properties> so we need the old
    // tag.
    CoulombFriction<double> friction =
        ParseCoulombFrictionFromDrakeCompliance(parent_element_name, node);
    props.AddProperty(geometry::internal::kMaterialGroup,
                      geometry::internal::kFriction, friction);
  } else {
    // We parsed friction from <drake:proximity_properties>; test for the
    // existence of <drake_compliance> and warn that it won't be used.
    if (node->FirstChildElement("drake_compliance")) {
      drake::log()->warn(fmt::format(
          "Drake contact parameters are fully specified by the "
          "<drake:proximity_properties> tag for the '{}' link. The "
          "<drake_compliance> tag is ignored. Consider removing  it.",
          parent_element_name));
    }
  }

  std::string geometry_name;
  if (!ParseStringAttribute(node, "name", &geometry_name)) {
    geometry_name = MakeGeometryName(parent_element_name + "_Collision", node);
  }

  geometry::GeometryInstance instance(T_element_to_link, std::move(shape),
                                      geometry_name);
  instance.set_proximity_properties(std::move(props));
  return instance;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
