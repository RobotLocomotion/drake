#include "drake/multibody/parsing/detail_urdf_geometry.h"

#include <iomanip>
#include <memory>
#include <ostream>
#include <set>
#include <sstream>
#include <stdexcept>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/filesystem.h"
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

UrdfMaterial AddMaterialToMaterialMap(const std::string& material_name,
                                      UrdfMaterial material,
                                      bool abort_if_name_clash,
                                      MaterialMap* materials) {
  DRAKE_DEMAND(materials != nullptr);
  // Determines if the material is already in the map.
  auto material_iter = materials->find(material_name);
  if (material_iter != materials->end()) {
    const UrdfMaterial& cached_material = material_iter->second;

    // Given a value (either rgba or diffuse_map) the following table indicates
    // the possible configurations and results:
    //
    //   Cached  |  Input  | Cached != Input |Desired Result
    //  ---------|---------|-----------------|----------------
    //   nullopt | nullopt |     false       | No error, They match
    //   nullopt |  val1   |      true       | ERROR
    //    val1   | nullopt |      true       | No error, not defining a value is
    //           |         |                 | compatible with a previous value.
    //    val1   |  val1   |     false       | No error, they match
    //    val1   |  val2   |      true       | ERROR, they don't match

    bool error = abort_if_name_clash;
    if (!error) {
      // Evaluate for diffuse map.

      // Note: Comparing two optional objects is convenient because it will
      // handle the case of one or the other being nullopt, both being nullopt,
      // and both being defined (using default operator== on the contained
      // type -- which we can use for diffuse map). However, the logic doesn't
      // exactly align with error modes. So, we use the optional comparison test
      // as the initial classification but "subtract" the one case that isn't
      // really an error (cached.has_value() and !input.has_value()).
      const std::optional<std::string> cached = cached_material.diffuse_map;
      const std::optional<std::string> input = material.diffuse_map;
      error = cached != input && !(cached.has_value() && !input.has_value());
    }
    if (!error) {
      // Evaluate for rgba. However, colors are equal to within a tolerance.
      const std::optional<Vector4d> cached = cached_material.rgba;
      const std::optional<Vector4d> input = material.rgba;
      if (cached.has_value() && input.has_value()) {
        if (cached.has_value()) {
          DRAKE_DEMAND(material.rgba.has_value());
          const Vector4d delta = *(cached_material.rgba) - *(material.rgba);
          error = delta.norm() > 1e-10;
        }
      } else {
        error = !cached.has_value() && input.has_value();
      }
    }

    if (error) {
      auto mat_descrip = [](const UrdfMaterial& mat) {
        std::string rgb_string =
            mat.rgba.has_value()
                ? fmt::format("RGBA: {}", mat.rgba->transpose())
                : "RGBA: None";
        std::string map_string =
            mat.diffuse_map.has_value()
                ? fmt::format("Diffuse map: {}", *(mat.diffuse_map))
                : "Diffuse map: None";
        return fmt::format("{}, {}", rgb_string, map_string);
      };
      throw std::runtime_error(fmt::format(
          "Material '{}' was previously defined.\n  - existing definition: "
          "{}\n  - new definition:      {}",
          material_name, mat_descrip(cached_material), mat_descrip(material)));
    }
  } else {
    // If no rgba color was defined, it defaults to *transparent* black.
    if (!material.rgba.has_value()) material.rgba = Vector4d(Vector4d::Zero());
    (*materials)[material_name] = std::move(material);
  }
  return (*materials)[material_name];
}

UrdfMaterial ParseMaterial(const XMLElement* node, bool name_required,
                           const PackageMap& package_map,
                           const std::string& root_dir,
                           MaterialMap* materials) {
  DRAKE_DEMAND(materials != nullptr);

  if (std::string(node->Name()) != "material") {
    throw std::runtime_error(std::string(
        fmt::format("Expected material element, got <{}>", node->Name())));
  }

  std::string name;
  ParseStringAttribute(node, "name", &name);
  if (name.empty() && name_required) {
    // Error condition: #1: name is required.
    throw std::runtime_error(
        fmt::format("Material tag on line {} is missing a required name",
                    node->GetLineNum()));
  }

  // Test for texture information.
  std::optional<std::string> texture_path;
  const XMLElement* texture_node = node->FirstChildElement("texture");
  if (texture_node) {
    std::string texture_name;
    if (ParseStringAttribute(texture_node, "filename", &texture_name) &&
        !texture_name.empty()) {
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
  }

  UrdfMaterial material{rgba, texture_path};

  if (!name.empty()) {
    // Error condition: #3.
    // If a name is *required*, then simply matching names should lead to an
    // error.
    material = AddMaterialToMaterialMap(name, material,
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
  // Rely on geometry::Shape to validate physical parameters.
  return std::make_unique<geometry::Box>(size(0), size(1), size(2));
}

std::unique_ptr<geometry::Shape> ParseSphere(const XMLElement* shape_node) {
  double r = 0;
  if (!ParseScalarAttribute(shape_node, "radius", &r)) {
    throw std::runtime_error("Missing sphere attribute: radius");
  }

  // Rely on geometry::Shape to validate physical parameters.
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
  // Rely on geometry::Shape to validate physical parameters.
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
  // Rely on geometry::Shape to validate physical parameters.
  return std::make_unique<geometry::Capsule>(r, l);
}

std::unique_ptr<geometry::Shape> ParseEllipsoid(const XMLElement* shape_node) {
  double axes[3];
  const char* names[] = {"a", "b", "c"};
  for (int i = 0; i < 3; ++i) {
    if (!ParseScalarAttribute(shape_node, names[i], &axes[i])) {
      throw std::runtime_error(
          fmt::format("Missing ellipsoid attribute: {}", names[i]));
    }
  }
  return std::make_unique<geometry::Ellipsoid>(axes[0], axes[1], axes[2]);
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

  // Rely on geometry::Shape to validate physical parameters.
  if (shape_node->FirstChildElement("drake:declare_convex")) {
    return std::make_unique<geometry::Convex>(resolved_filename, scale);
  } else {
    return std::make_unique<geometry::Mesh>(resolved_filename, scale);
  }
}

std::unique_ptr<geometry::Shape> ParseGeometry(const XMLElement* node,
                                               const PackageMap& package_map,
                                               const std::string& root_dir) {
  if (auto child_node = node->FirstChildElement("box"); child_node) {
    return ParseBox(child_node);
  } else if (child_node = node->FirstChildElement("sphere"); child_node) {
    return ParseSphere(child_node);
  } else if (child_node = node->FirstChildElement("cylinder"); child_node) {
    return ParseCylinder(child_node);
  } else if (child_node = node->FirstChildElement("capsule");
             child_node) {
    // TODO(SeanCurtis-TRI): This should *not* be <capsule>. It should be
    //  <drake:capsule>. <capsule> is not in the spec
    //  http://wiki.ros.org/urdf/XML/link. And even there has been a three-year
    //  debate about adding it into ros (still unresolved):
    //  https://github.com/ros/urdfdom_headers/pull/24
    //  Given that this is a tag that is *not* in the spec, it requires the
    //  namespace.
    //  As a footnote, bullet does support it:
    //  https://github.com/bulletphysics/bullet3/blob/master/data/capsule.urdf
    //  and we have a number of legacy files that have <capsule> declarations
    //  in them.
    return ParseCapsule(child_node);
  } else if (child_node = node->FirstChildElement("mesh"); child_node) {
    return ParseMesh(child_node, package_map, root_dir);
  } else if (child_node = node->FirstChildElement("drake:ellipsoid");
             child_node) {
    return ParseEllipsoid(child_node);
  }

  throw std::runtime_error(fmt::format(
      "Warning: geometry element on line {} "
      "does not have a recognizable shape type", node->GetLineNum()));
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

  // The empty property set relies on downstream consumer default behavior.
  geometry::IllustrationProperties properties;

  const XMLElement* material_node = node->FirstChildElement("material");
  if (material_node) {
    UrdfMaterial material =
        ParseMaterial(material_node, false /* name required */, package_map,
                      root_dir, materials);
    if (material.rgba) {
      properties = geometry::MakePhongIllustrationProperties(*(material.rgba));
    }
    if (material.diffuse_map) {
      properties.AddProperty("phong", "diffuse_map", *(material.diffuse_map));
    }
  }

  // TODO(SeanCurtis-TRI): Including this property in illustration properties is
  //  a bit misleading; it isn't used by illustration, but we're not currently
  //  parsing illustration and perception properties separately. So, we stash
  //  them in the illustration properties relying on it to be ignored by
  //  illustration consumers but copied over to the perception properties.
  const char* kAcceptingTag = "drake:accepting_renderer";
  const XMLElement* accepting_node = node->FirstChildElement(kAcceptingTag);
  if (accepting_node) {
    std::set<std::string> accepting_names;
    while (accepting_node) {
      std::string name;
      if (!ParseStringAttribute(accepting_node, "name", &name)) {
        throw std::runtime_error(
            fmt::format("<{}}> tag given on line {} without any name",
                        kAcceptingTag, accepting_node->GetLineNum()));
      }
      accepting_names.insert(name);
      accepting_node = accepting_node->NextSiblingElement(kAcceptingTag);
    }
    DRAKE_DEMAND(accepting_names.size() > 0);
    properties.AddProperty("renderer", "accepting", std::move(accepting_names));
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

// TODO(SeanCurtis-TRI): Remove all of this legacy parsing code based on
//  issue #12598.

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
      static const logging::Warn log_once(
          "At least one of your URDF files has specified the <youngs_modulus> "
          "tag under the <drake_compliance> tag. Drake no longer makes use of "
          "that tag and all instances will be ignored.");
    }

    if (compliant_node->FirstChildElement("dissipation")) {
      static const logging::Warn log_once(
          "At least one of your URDF files has specified the <dissipation> "
          "tag under the <drake_compliance> tag. Drake no longer makes use of "
          "that tag and all instances will be ignored.");
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
    static const logging::Warn log_once(
        "At least one of your URDF files has specified the 'group' attribute "
        "on the <collision> tag. Drake doesn't make use of that attribute and "
        "all instances will be ignored.");
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
      std::optional<double> result;
      const XMLElement* value_node =
          drake_element->FirstChildElement(element_name);
      if (value_node != nullptr) {
        double value{};
        if (ParseScalarAttribute(value_node, "value", &value)) {
          result = value;
        } else {
          throw std::runtime_error(
              fmt::format("Unable to read the 'value' attribute for the <{}> "
                          "tag on line {}",
                          element_name, value_node->GetLineNum()));
        }
      }
      return result;
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

  // TODO(SeanCurtis-TRI): Remove all of this legacy parsing code based on
  //  issue #12598.
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
      static const logging::Warn log_once(
          "At least one of your URDF files has specified collision properties "
          "using both <drake:proximity_properties> and <drake_compliance> "
          "tags. The <drake_compliance> tag is ignored in this case and is "
          "generally dispreferred. Consider removing all instances of "
          "<drake_compliance> in favor of <drake:proximity_properties>.");
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
