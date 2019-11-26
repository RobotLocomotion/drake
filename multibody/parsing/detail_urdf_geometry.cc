#include "drake/multibody/parsing/detail_urdf_geometry.h"

#include <iomanip>
#include <memory>
#include <ostream>
#include <sstream>
#include <stdexcept>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/filesystem.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/geometry_roles.h"
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

// Adds a material to the supplied @materials map.
//
// @param[in] material_name A human-understandable name of the material.
//
// @param[in] material The definition of the URDF material to associate with the
// name.
//
// @param[in] abort_if_name_clash If true, this method will abort if
// @p material_name is already in @p materials regardless of whether the
// material is the same. If false, this method will abort if
// @p material_name is already in @p materials and material definition doesn't
// match.
//
// @param[out] materials A pointer to the map in which to store the material.
// This cannot be nullptr.
void AddMaterialToMaterialMap(const std::string& material_name,
                              UrdfMaterial material, bool abort_if_name_clash,
                              MaterialMap* materials) {
  DRAKE_DEMAND(materials);
  // Determines if the material is already in the map.
  auto material_iter = materials->find(material_name);
  if (material_iter != materials->end()) {
    const UrdfMaterial& cached_material = material_iter->second;

    // To be equivalent, they must define the same quantities and the defined
    // values must match.
    bool error =
        abort_if_name_clash ||
        (cached_material.rgba.has_value() != material.rgba.has_value()) ||
        (cached_material.diffuse_map != material.diffuse_map);
    if (!error) {
      // The only thing that hasn't been tested is that the rgba values match
      // (if they exist). We already know that both cached and input materials
      // match w.r.t. *having* rgba data.
      if (cached_material.rgba.has_value()) {
        DRAKE_DEMAND(material.rgba.has_value());
        const Vector4d delta = *(cached_material.rgba) - *(material.rgba);
        error = error && delta.norm() > 1e-10;
      }
    }
    if (error) {
      auto mat_descrip = [](const UrdfMaterial& mat) {
        std::string rgb_string =
            mat.rgba.has_value()
                ? fmt::format("RGBA: {}", mat.rgba->transpose())
                : std::string("RGBA: None");
        std::string map_string =
            mat.diffuse_map.has_value()
                ? fmt::format("Diffuse map: {}", *(mat.diffuse_map))
                : std::string("Diffuse map: None");
        return fmt::format("{}, {}", rgb_string, map_string);
      };
      throw std::runtime_error(fmt::format(
          "Material '{}' was previously defined.\n  - existing definition: "
          "{}\n  - new definition:      {}",
          material_name, mat_descrip(cached_material), mat_descrip(material)));
    }
  } else {
    (*materials)[material_name] = std::move(material);
  }
}

}  // namespace

UrdfMaterial ParseMaterial(const XMLElement* node, bool name_required,
                           const PackageMap& package_map,
                           const std::string& root_dir,
                           MaterialMap* materials) {
  DRAKE_DEMAND(materials != nullptr);

  if (std::string(node->Name()) != "material") {
    throw std::runtime_error(
        std::string("Expected material element, got ") + node->Name());
  }

  std::string name;
  ParseStringAttribute(node, "name", &name);
  if (name.empty() && name_required) {
    // Error condition: #1: name is required.
    throw std::runtime_error("Material tag is missing a name.");
  }

  // Test for texture information.
  std::optional<std::string> texture_path;
  const XMLElement* texture_node = node->FirstChildElement("texture");
  if (texture_node) {
    std::string texture_name;
    if (ParseStringAttribute(texture_node, "filename", &texture_name) &&
        !texture_name.empty()) {
      // Note: the validity of the path is validated if and when this property
      // is used by some downstream render engine.
      texture_path = ResolveUri(texture_name, package_map, root_dir);
      if (texture_path->empty()) {
        // Error condition: #4. File specified, but the resource is not
        // available.
        throw std::runtime_error(fmt::format(
            "Unable to locate the texture file defined on line {}: {}",
            texture_node->GetLineNum(), texture_name));
      }
    }
  }

  // Now test for color information.
  std::optional<Vector4d> rgba = std::nullopt;
  // Checks and remembers whether a "color" child element exists. If so,
  // parses the color value.
  const XMLElement* color_node = node->FirstChildElement("color");
  if (color_node) {
    Vector4d rgba_value;
    if (!ParseVectorAttribute(color_node, "rgba", &rgba_value)) {
      throw std::runtime_error(
          fmt::format("Failed to parse 'rgba' attribute of <color> on line {}",
                      color_node->GetLineNum()));
    }
    rgba = rgba_value;
  }

  if (!rgba && !texture_path) {
    if (!name.empty() && materials->find(name) == materials->end()) {
      // Error condition: #2: name with no properties has not been previously
      // defined.
      throw std::runtime_error(
          fmt::format("Material '{}' defined on line {} not previously "
                      "defined, but has no color or texture information.",
                      name, node->GetLineNum()));
    }
    // Can't assign the const value type to an optional<Vector4d> directly.
    rgba = Vector4d(Vector4d::Zero());  // Defaults to black.
  }

  UrdfMaterial material{rgba, texture_path};

  if (!name.empty()) {
    // Error condition: #3.
    // If a name is *required*, then simply matching names should lead to an
    // error.
    AddMaterialToMaterialMap(name, material,
        name_required /* abort_if_name_clash */, materials);
  }
  return material;
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

std::unique_ptr<geometry::Shape> ParseMesh(
    const XMLElement* shape_node, const PackageMap& package_map,
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

std::unique_ptr<geometry::Shape> ParseGeometry(
    const XMLElement* node, const PackageMap& package_map,
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
    return ParseMesh(node->FirstChildElement("mesh"), package_map,
                     root_dir);
  }

  throw std::runtime_error("Warning: geometry element "
                           "has an unknown type and will be ignored.");
}

std::string MakeGeometryName(const std::string& basename,
                             const XMLElement* node) {
  using std::setfill;
  using std::setw;
  using std::hex;

  // Append the address spelled like "@0123456789abcdef".
  intptr_t address = reinterpret_cast<intptr_t>(node);
  std::ostringstream result;
  result << basename << '@' << setfill('0') << setw(16) << hex << address;
  return result.str();
}

}  // namespace

// Parses a "visual" element in @p node.
geometry::GeometryInstance ParseVisual(
    const std::string& parent_element_name,
    const PackageMap& package_map,
    const std::string& root_dir, const XMLElement* node,
    MaterialMap* materials) {
  if (std::string(node->Name()) != "visual") {
    throw std::runtime_error(
        "In link " + parent_element_name +
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

  // The empty set relies on consumer defaults.
  geometry::IllustrationProperties properties;

  const XMLElement* material_node = node->FirstChildElement("material");
  if (material_node) {
    UrdfMaterial material =
        ParseMaterial(material_node, false, package_map, root_dir, materials);
    if (material.rgba) {
      properties = geometry::MakePhongIllustrationProperties(*(material.rgba));
    }
    if (material.diffuse_map) {
      properties.AddProperty("phong", "diffuse_map", *(material.diffuse_map));
    }
  }

  std::string geometry_name;
  if (!ParseStringAttribute(node, "name", &geometry_name)) {
    geometry_name = MakeGeometryName(parent_element_name + "_Visual", node);
  }

  auto instance = geometry::GeometryInstance(
      T_element_to_link, std::move(shape), geometry_name);
  instance.set_illustration_properties(properties);
  return instance;
}

// Parses a "collision" element in @p node.
//
// @param[out] friction Coulomb friction for the associated geometry.
geometry::GeometryInstance ParseCollision(
    const std::string& parent_element_name,
    const PackageMap& package_map,
    const std::string& root_dir, const XMLElement* node,
    CoulombFriction<double>* friction) {
  if (std::string(node->Name()) != "collision") {
    throw std::runtime_error(
        "In link " + parent_element_name +
        " expected collision element, got " + node->Name());
  }

  // Ensures there is a geometry child element. Since this is a required
  // element, throws an exception if a geometry element does not exist.
  const XMLElement* geometry_node = node->FirstChildElement("geometry");
  if (!geometry_node) {
    throw std::runtime_error("Link " + parent_element_name +
                             " has a collision element without geometry.");
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

  *friction = default_friction();
  const XMLElement* compliant_node =
      node->FirstChildElement("drake_compliance");
  if (compliant_node) {
    double static_friction{-1};
    double dynamic_friction{-1};
    bool static_friction_present = false;
    bool dynamic_friction_present = false;

    const XMLElement* friction_node =
        compliant_node->FirstChildElement("static_friction");
    if (friction_node) {
      static_friction_present = true;
      if (friction_node->QueryDoubleText(&static_friction)) {
        throw std::runtime_error(
            "Unable to parse static_friction for link " + parent_element_name);
      }
    }

    friction_node = compliant_node->FirstChildElement("dynamic_friction");
    if (friction_node) {
      dynamic_friction_present = true;
      if (friction_node->QueryDoubleText(&dynamic_friction)) {
        throw std::runtime_error(
            "Unable to parse dynamic_friction for link " + parent_element_name);
      }
    }

    if (static_friction_present != dynamic_friction_present) {
        throw std::runtime_error(
            "Link " + parent_element_name +
            ": When specifying coefficient of friction, "
            "both static and dynamic coefficients must be defined");
    }

    if (static_friction_present) {
      *friction = CoulombFriction<double>(static_friction, dynamic_friction);
    }

    if (compliant_node->FirstChildElement("youngs_modulus")) {
      drake::log()->warn("Ignoring youngs_modulus for link " +
                         parent_element_name);
    }

    if (compliant_node->FirstChildElement("dissipation")) {
      drake::log()->warn("Ignoring dissipation for link " +
                         parent_element_name);
    }
  }

  std::string geometry_name;
  if (!ParseStringAttribute(node, "name", &geometry_name)) {
    geometry_name = MakeGeometryName(parent_element_name + "_Collision", node);
  }

  return geometry::GeometryInstance(T_element_to_link, std::move(shape),
                                    geometry_name);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
