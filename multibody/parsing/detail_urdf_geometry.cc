#include "drake/multibody/parsing/detail_urdf_geometry.h"

#include <filesystem>
#include <iomanip>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>

#include "drake/common/diagnostic_policy.h"
#include "drake/common/drake_assert.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/parsing/detail_tinyxml.h"
#include "drake/multibody/parsing/detail_tinyxml2_diagnostic.h"

namespace drake {
namespace multibody {
namespace internal {

using Eigen::Vector3d;
using Eigen::Vector4d;
using math::RigidTransformd;
using tinyxml2::XMLElement;

using drake::internal::DiagnosticPolicy;

UrdfMaterial AddMaterialToMaterialMap(
    const drake::internal::DiagnosticPolicy& policy,
    const std::string& material_name,
    UrdfMaterial material,
    bool error_if_name_clash,
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

    bool error = error_if_name_clash;
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
                ? fmt::format("RGBA: {}", fmt_eigen(mat.rgba->transpose()))
                : "RGBA: None";
        std::string map_string =
            mat.diffuse_map.has_value()
                ? fmt::format("Diffuse map: {}", *(mat.diffuse_map))
                : "Diffuse map: None";
        return fmt::format("{}, {}", rgb_string, map_string);
      };
      policy.Error(
          fmt::format("Material '{}' was previously defined.\n"
                      "  - existing definition: {}\n"
                      "  - new definition:      {}",
                      material_name, mat_descrip(cached_material),
                      mat_descrip(material)));
      return {};
    }
  } else {
    // If no rgba color was defined, it defaults to *transparent* black.
    if (!material.rgba.has_value()) material.rgba = Vector4d(Vector4d::Zero());
    (*materials)[material_name] = std::move(material);
  }
  return (*materials)[material_name];
}

UrdfMaterial ParseMaterial(const TinyXml2Diagnostic& diagnostic,
                           const tinyxml2::XMLElement* node, bool name_required,
                           const PackageMap& package_map,
                           const std::string& root_dir,
                           MaterialMap* materials) {
  DRAKE_DEMAND(materials != nullptr);

  if (std::string(node->Name()) != "material") {
    diagnostic.Error(*node, fmt::format("Expected material element, got <{}>",
                                        node->Name()));
    return {};
  }

  std::string name;
  ParseStringAttribute(node, "name", &name);
  if (name.empty() && name_required) {
    // Error condition: #1: name is required.
    diagnostic.Error(*node, "Material tag is missing a required name");
    return {};
  }

  // Test for texture information.
  std::optional<std::string> texture_path;
  const XMLElement* texture_node = node->FirstChildElement("texture");
  if (texture_node) {
    // TODO(rpoyner-tri): error for empty texture tag?
    std::string texture_name;
    if (ParseStringAttribute(texture_node, "filename", &texture_name) &&
        !texture_name.empty()) {
      texture_path = ResolveUri(diagnostic.MakePolicyForNode(texture_node),
                                texture_name, package_map, root_dir);
      if (texture_path->empty()) {
        // ResolveUri already emitted an error message.
        return {};
      }
    }
  }

  // Now test for color information.
  std::optional<Vector4d> rgba = std::nullopt;
  const XMLElement* color_node = node->FirstChildElement("color");
  if (color_node) {
    Vector4d rgba_value;
    if (!ParseVectorAttribute(color_node, "rgba", &rgba_value)) {
      diagnostic.Error(*color_node,
                       "Failed to parse 'rgba' attribute of <color>}");
      return {};
    }
    rgba = rgba_value;
  }

  if (!rgba && !texture_path) {
    if (!name.empty() && materials->find(name) == materials->end()) {
      // Error condition: #2: name with no properties has not been previously
      // defined.
      diagnostic.Error(
          *node, fmt::format("Material '{}' not previously defined, but has no"
                             " color or texture information.", name));
      return {};
    }
  }

  UrdfMaterial material{rgba, texture_path};

  if (!name.empty()) {
    // Error condition: #3.
    // If a name is *required*, then simply matching names should lead to an
    // error.
    material = AddMaterialToMaterialMap(
        diagnostic.MakePolicyForNode(node), name, material,
        name_required /* error_if_name_clash */, materials);
  }
  return material;
}

namespace {

std::unique_ptr<geometry::Shape> ParseBox(
    const TinyXml2Diagnostic& diagnostic,
    const XMLElement* shape_node) {
  Eigen::Vector3d size = Eigen::Vector3d::Zero();
  if (!ParseVectorAttribute(shape_node, "size", &size)) {
    diagnostic.Error(*shape_node, "Missing box attribute: size");
    return {};
  }
  // Rely on geometry::Shape to validate physical parameters.
  return std::make_unique<geometry::Box>(size(0), size(1), size(2));
}

std::unique_ptr<geometry::Shape> ParseSphere(
    const TinyXml2Diagnostic& diagnostic,
    const XMLElement* shape_node) {
  double r = 0;
  if (!ParseScalarAttribute(shape_node, "radius", &r,
                            diagnostic.MakePolicyForNode(shape_node))) {
    diagnostic.Error(*shape_node, "Missing sphere attribute: radius");
    return {};
  }

  // Rely on geometry::Shape to validate physical parameters.
  return std::make_unique<geometry::Sphere>(r);
}

std::unique_ptr<geometry::Shape> ParseCylinder(
    const TinyXml2Diagnostic& diagnostic,
    const XMLElement* shape_node) {
  double r = 0;
  if (!ParseScalarAttribute(shape_node, "radius", &r,
                            diagnostic.MakePolicyForNode(shape_node))) {
    diagnostic.Error(*shape_node, "Missing cylinder attribute: radius");
    return {};
  }

  double l = 0;
  if (!ParseScalarAttribute(shape_node, "length", &l,
                            diagnostic.MakePolicyForNode(shape_node))) {
    diagnostic.Error(*shape_node, "Missing cylinder attribute: length");
    return {};
  }
  // Rely on geometry::Shape to validate physical parameters.
  return std::make_unique<geometry::Cylinder>(r, l);
}

std::unique_ptr<geometry::Shape> ParseCapsule(
    const TinyXml2Diagnostic& diagnostic,
    const XMLElement* shape_node) {
  double r = 0;
  if (!ParseScalarAttribute(shape_node, "radius", &r,
                            diagnostic.MakePolicyForNode(shape_node))) {
    diagnostic.Error(*shape_node, "Missing capsule attribute: radius");
    return {};
  }

  double l = 0;
  if (!ParseScalarAttribute(shape_node, "length", &l,
                            diagnostic.MakePolicyForNode(shape_node))) {
    diagnostic.Error(*shape_node, "Missing capsule attribute: length");
    return {};
  }
  // Rely on geometry::Shape to validate physical parameters.
  return std::make_unique<geometry::Capsule>(r, l);
}

std::unique_ptr<geometry::Shape> ParseEllipsoid(
    const TinyXml2Diagnostic& diagnostic,
    const XMLElement* shape_node) {
  double axes[3];
  const char* names[] = {"a", "b", "c"};
  for (int i = 0; i < 3; ++i) {
    if (!ParseScalarAttribute(shape_node, names[i], &axes[i],
                              diagnostic.MakePolicyForNode(shape_node))) {
      diagnostic.Error(*shape_node, fmt::format(
                           "Missing ellipsoid attribute: '{}'", names[i]));
      return {};
    }
  }
  return std::make_unique<geometry::Ellipsoid>(axes[0], axes[1], axes[2]);
}

std::unique_ptr<geometry::Shape> ParseMesh(const TinyXml2Diagnostic& diagnostic,
                                           const XMLElement* shape_node,
                                           const PackageMap& package_map,
                                           const std::string& root_dir) {
  std::string filename;
  if (!ParseStringAttribute(shape_node, "filename", &filename)) {
    diagnostic.Error(*shape_node, "Mesh element has no filename tag");
    return {};
  }

  const std::string resolved_filename = ResolveUri(
      diagnostic.MakePolicyForNode(shape_node), filename, package_map,
      root_dir);
  if (resolved_filename.empty()) {
    // ResolveUri already emitted an error message.
    return {};
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
      diagnostic.Error(*shape_node, "Drake meshes only support isotropic"
                       " scaling. Therefore all three scaling factors must be"
                       " exactly equal.");
      return {};
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

std::unique_ptr<geometry::Shape> ParseGeometry(
    const TinyXml2Diagnostic& diagnostic,
    const XMLElement* node,
    const PackageMap& package_map,
    const std::string& root_dir) {
  if (auto child_node = node->FirstChildElement("box"); child_node) {
    return ParseBox(diagnostic, child_node);
  } else if (child_node = node->FirstChildElement("sphere"); child_node) {
    return ParseSphere(diagnostic, child_node);
  } else if (child_node = node->FirstChildElement("cylinder"); child_node) {
    return ParseCylinder(diagnostic, child_node);
  } else if (child_node = node->FirstChildElement("capsule");
             child_node) {
    // Accepting the <capsule> is non-standard:
    // http://wiki.ros.org/urdf/XML/link. And even there has been a long debate
    // about adding it into ros (still unresolved):
    // https://github.com/ros/urdfdom_headers/pull/24
    // As a footnote, bullet does support it:
    // https://github.com/bulletphysics/bullet3/blob/master/data/capsule.urdf
    // and we have a number of legacy files that have <capsule> declarations
    // in them.
    // TODO(rpoyner-tri): If the spec treatment of <capsule> changes, this
    // parse should be updated to match.
    return ParseCapsule(diagnostic, child_node);
  } else if (child_node = node->FirstChildElement("drake:capsule");
             child_node) {
    // We also accept <drake:capsule>, both as a hedge against changes in the
    // standard, and for similarity with drake SDFormat extensions.
    return ParseCapsule(diagnostic, child_node);
  } else if (child_node = node->FirstChildElement("mesh"); child_node) {
    return ParseMesh(diagnostic, child_node, package_map, root_dir);
  } else if (child_node = node->FirstChildElement("drake:ellipsoid");
             child_node) {
    return ParseEllipsoid(diagnostic, child_node);
  }

  diagnostic.Error(*node, "Warning: geometry element does not have a"
                   " recognizable shape type");
  return {};
}

// The goal here is to invent a name that will be unique within the enclosing
// body (i.e., link). To be as useful as possible, we'll use the shape name as
// the geometry name, and then tack on a number if necessary to be unique.
//
// TODO(jwnimmer-tri) The "tack on a number" heuristic might fail if the user
// mixes named and unnamed geometry within the same link and they happen to
// use the shape name. We should probably try to handle this in the outer
// parsing loop during ParseBody (by detecting the name collision). For now,
// though, that doesn't seem worth dealing with yet.
std::string MakeDefaultGeometryName(
    const drake::internal::DiagnosticPolicy& policy,
    const geometry::Shape& shape,
    const std::unordered_set<std::string>& geometry_names,
    int numeric_suffix_limit) {
  const std::string shape_name = geometry::ShapeName(shape).name();
  if (geometry_names.count(shape_name) == 0) {
    return shape_name;
  }
  for (int i = 1; i < numeric_suffix_limit; ++i) {
    std::string guess = fmt::format("{}{}", shape_name, i);
    if (geometry_names.count(guess) == 0) {
      return guess;
    }
  }
  policy.Error("Too many identical geometries with default names.");
  return {};
}

}  // namespace

// Parses a "visual" element in @p node.
std::optional<geometry::GeometryInstance> ParseVisual(
    const TinyXml2Diagnostic& diagnostic,
    const std::string& parent_element_name,
    const PackageMap& package_map,
    const std::string& root_dir, const tinyxml2::XMLElement* node,
    MaterialMap* materials, std::unordered_set<std::string>* geometry_names,
    int numeric_name_suffix_limit) {
  if (std::string(node->Name()) != "visual") {
    diagnostic.Error(*node, fmt::format(
                         "In link {} expected visual element, got {}",
                         parent_element_name, node->Name()));
    return {};
  }

  // Ensures there is a geometry child element. Since this is a required
  // element, emits an error if a geometry element does not exist.
  const XMLElement* geometry_node = node->FirstChildElement("geometry");
  if (!geometry_node) {
    diagnostic.Error(*node, fmt::format(
                         "Link {} has a visual element without geometry.",
                         parent_element_name));
    return {};
  }

  // Obtains the reference frame of the visualization relative to the reference
  // frame of the rigid body that is being visualized. It defaults to identity
  // if no transform is specified.
  RigidTransformd T_element_to_link;
  const XMLElement* origin = node->FirstChildElement("origin");
  if (origin) {
    T_element_to_link = OriginAttributesToTransform(origin);
  }

  std::unique_ptr<geometry::Shape> shape =
      ParseGeometry(diagnostic, geometry_node, package_map, root_dir);
  if (!shape) {
    // Error should have already been emitted.
    return {};
  }

  // The empty property set relies on downstream consumer default behavior.
  geometry::IllustrationProperties properties;

  const XMLElement* material_node = node->FirstChildElement("material");
  if (material_node) {
    UrdfMaterial material =
        ParseMaterial(diagnostic, material_node, false /* name required */,
                      package_map, root_dir, materials);
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
        diagnostic.Error(*accepting_node, fmt::format(
                             "<{}> tag given without any name",
                             kAcceptingTag));
        return {};
      }
      accepting_names.insert(name);
      accepting_node = accepting_node->NextSiblingElement(kAcceptingTag);
    }
    DRAKE_DEMAND(accepting_names.size() > 0);
    properties.AddProperty("renderer", "accepting", std::move(accepting_names));
  }

  std::string geometry_name;
  if (!ParseStringAttribute(node, "name", &geometry_name)) {
    geometry_name = MakeDefaultGeometryName(diagnostic.MakePolicyForNode(node),
                                            *shape, *geometry_names,
                                            numeric_name_suffix_limit);
    if (geometry_name.empty()) {
      // Error should already have been emitted.
      return {};
    }
  }
  geometry_names->insert(geometry_name);

  auto instance = geometry::GeometryInstance(T_element_to_link,
                                             std::move(shape), geometry_name);
  instance.set_illustration_properties(properties);
  return instance;
}

namespace {

// TODO(SeanCurtis-TRI): Remove all of this legacy parsing code based on
//  issue #12598.

// This is the backwards-compatible fallback for defining friction; it reads
// the soon-to-be-deprecated <drake_compliance> tag for data. It emits errors
// for malformed values or returns a friction (either the valid friction
// defined in the tag or the default).
//
// It incidentally propagates some warnings about unused tags from the rigid
// body tree days.
CoulombFriction<double> ParseCoulombFrictionFromDrakeCompliance(
    const TinyXml2Diagnostic& diagnostic,
    const std::string& parent_element_name, const XMLElement* node) {
  const XMLElement* compliant_node =
      node->FirstChildElement("drake_compliance");
  if (compliant_node) {
    // TODO(SeanCurtis-TRI): Ultimately, we want to kill <drake_compliance>
    //  and these will go along with it. These values are only used in rigid
    //  body tree; with no real expectation we'll re-use them in MBP.
    if (compliant_node->FirstChildElement("youngs_modulus")) {
      diagnostic.Warning(
          *compliant_node,
          "A link has specified the <youngs_modulus> tag under the"
          " <drake_compliance> tag. Drake no longer makes use of that tag and"
          " all instances will be ignored.");
    }

    if (compliant_node->FirstChildElement("dissipation")) {
      diagnostic.Warning(
          *compliant_node,
          "A link has specified the <dissipation> tag under the"
          " <drake_compliance> tag. Drake no longer makes use of that tag and"
          " all instances will be ignored.");
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
        diagnostic.Error(*friction_node, fmt::format(
                             "Unable to parse static_friction for link {}",
                             parent_element_name));
        return default_friction();
      }
    }

    friction_node = compliant_node->FirstChildElement("dynamic_friction");
    if (friction_node) {
      dynamic_friction_present = true;
      if (friction_node->QueryDoubleText(&dynamic_friction)) {
        diagnostic.Error(*friction_node, fmt::format(
                             "Unable to parse dynamic_friction for link {}",
                             parent_element_name));
        return default_friction();
      }
    }

    if (static_friction_present != dynamic_friction_present) {
      diagnostic.Error(*compliant_node, fmt::format(
                           "Link '{}': When specifying coefficient of friction,"
                           " both static and dynamic coefficients must be"
                           " defined", parent_element_name));
      return default_friction();
    }

    if (static_friction_present) {
      return CoulombFriction<double>(static_friction, dynamic_friction);
    }
  }
  return default_friction();
}

}  // namespace

// Parses a "collision" element in @p node.
//
// @param[out] friction Coulomb friction for the associated geometry.
std::optional<geometry::GeometryInstance> ParseCollision(
    const TinyXml2Diagnostic& diagnostic,
    const std::string& parent_element_name,
    const PackageMap& package_map,
    const std::string& root_dir, const tinyxml2::XMLElement* node,
    std::unordered_set<std::string>* geometry_names,
    int numeric_name_suffix_limit) {
  if (std::string(node->Name()) != "collision") {
    diagnostic.Error(*node, fmt::format(
                         "In link '{}' expected collision element, got {}",
                         parent_element_name, node->Name()));
    return {};
  }
  // Seen in the ROS urdfdom XSD Schema.
  // See https://github.com/ros/urdfdom/blob/dbecca0/xsd/urdf.xsd
  diagnostic.WarnUnsupportedElement(*node, "verbose");

  // Ensures there is a geometry child element. Since this is a required
  // element, emits an error if a geometry element does not exist.
  const XMLElement* geometry_node = node->FirstChildElement("geometry");
  if (!geometry_node) {
    diagnostic.Error(*node, fmt::format(
                         "Link '{}' has a collision element without geometry",
                         parent_element_name));
    return {};
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
    diagnostic.Warning(
        *node,
        "A link has specified the 'group' attribute on the <collision> tag."
        " Drake doesn't make use of that attribute and all instances will be"
        " ignored.");
  }

  std::unique_ptr<geometry::Shape> shape =
      ParseGeometry(diagnostic, geometry_node, package_map, root_dir);
  if (!shape) {
    // Error should have already been emitted.
    return {};
  }

  // Parse the properties from <drake:proximity_properties>.
  geometry::ProximityProperties props;
  const XMLElement* drake_element =
      node->FirstChildElement("drake:proximity_properties");
  if (drake_element) {
    auto read_double =
        [drake_element, &diagnostic](const char* element_name)
        -> std::optional<double> {
      const XMLElement* value_node =
          drake_element->FirstChildElement(element_name);
      if (value_node != nullptr) {
        double value{};
        if (ParseScalarAttribute(value_node, "value", &value,
                                 diagnostic.MakePolicyForNode(value_node))) {
          return value;
        } else {
          diagnostic.Error(*value_node, fmt::format(
                               "Unable to read the 'value' attribute for the"
                               " <{}> tag", element_name));
          // Fall through to failure return.
        }
      }
      return std::nullopt;
    };

    const XMLElement* const rigid_element =
        drake_element->FirstChildElement("drake:rigid_hydroelastic");
    const XMLElement* const compliant_element =
        drake_element->FirstChildElement("drake:compliant_hydroelastic");

    // TODO(16229): Remove this ad-hoc input sanitization when we resolve
    //  issue 16229 "Diagnostics for unsupported SDFormat and URDF stanzas."
    const XMLElement* const no_longer_supported =
        drake_element->FirstChildElement("drake:soft_hydroelastic");
    if (no_longer_supported) {
      diagnostic.Error(*no_longer_supported, "Collision geometry uses the tag"
                 " <drake:soft_hydroelastic>, which is no longer supported."
                 " Please change it to <drake:compliant_hydroelastic>.");
      return {};
    }

    if (rigid_element && compliant_element) {
      diagnostic.Error(
          *drake_element, fmt::format(
              "Collision geometry has defined mutually-exclusive tags"
              " <drake:rigid_hydroelastic> and <drake:compliant_hydroelastic>"
              " on lines {} and {}, respectively. Only one can be provided.",
              rigid_element->GetLineNum(), compliant_element->GetLineNum()));
      return {};
    }

    props = ParseProximityProperties(
        diagnostic.MakePolicyForNode(drake_element), read_double,
        rigid_element != nullptr, compliant_element != nullptr);
  }

  // TODO(SeanCurtis-TRI): Remove all of this legacy parsing code based on
  //  issue #12598.
  // Now test to see how we should handle a potential <drake_compliance> tag.
  if (!props.HasProperty(geometry::internal::kMaterialGroup,
                         geometry::internal::kFriction)) {
    // We have no friction from <drake:proximity_properties> so we need the old
    // tag.
    CoulombFriction<double> friction = ParseCoulombFrictionFromDrakeCompliance(
        diagnostic, parent_element_name, node);
    props.AddProperty(geometry::internal::kMaterialGroup,
                      geometry::internal::kFriction, friction);
  } else {
    // We parsed friction from <drake:proximity_properties>; test for the
    // existence of <drake_compliance> and warn that it won't be used.
    if (node->FirstChildElement("drake_compliance")) {
      diagnostic.Warning(
          *node,
          "A link has specified collision properties using both"
          " <drake:proximity_properties> and <drake_compliance> tags. The"
          " <drake_compliance> tag is ignored in this case and is generally"
          " dispreferred. Consider removing all instances of <drake_compliance>"
          " in favor of <drake:proximity_properties>.");
    }
  }

  std::string geometry_name;
  if (!ParseStringAttribute(node, "name", &geometry_name)) {
    geometry_name = MakeDefaultGeometryName(diagnostic.MakePolicyForNode(node),
                                            *shape, *geometry_names,
                                            numeric_name_suffix_limit);
    if (geometry_name.empty()) {
      // Error should already have been emitted.
      return {};
    }
  }
  geometry_names->insert(geometry_name);

  geometry::GeometryInstance instance(T_element_to_link, std::move(shape),
                                      geometry_name);
  instance.set_proximity_properties(std::move(props));
  return instance;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
