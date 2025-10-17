#include "drake/multibody/parsing/detail_sdf_geometry.h"

#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Element.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Param.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>

#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_ignition.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/parsing/detail_sdf_diagnostic.h"
#include "drake/multibody/plant/coulomb_friction.h"

namespace drake {
namespace multibody {
namespace internal {

using Eigen::Vector3d;
using std::make_unique;
using std::set;
using std::string;

using drake::internal::DiagnosticPolicy;
using geometry::GeometryInstance;
using geometry::GeometryProperties;
using geometry::IllustrationProperties;
using geometry::PerceptionProperties;
using geometry::ProximityProperties;
using math::RigidTransformd;

namespace {

// Helper to return the value of a child of `element` named `child_name`.
// A std::runtime_error is thrown if the `<child_name>` tag is missing from the
// SDF file and no `default_value` is supplied, or the tag has a bad or missing
// value.
template <typename T>
std::optional<T> GetChildElementValue(
    const SDFormatDiagnostic& diagnostic, const sdf::ElementConstPtr element,
    const std::string& child_name,
    const std::optional<T>& default_value = std::nullopt) {
  // TODO(amcastro-tri): unit tests for different error paths are needed.

  if (!element->HasElement(child_name)) {
    if (default_value) {
      return *default_value;
    }
    diagnostic.Error(
        element, fmt::format("Element <{}> is required within element <{}>.",
                             child_name, element->GetName()));
    return std::nullopt;
  }
  std::pair<T, bool> value_pair = element->Get<T>(child_name, T());
  // We already know that HasElement() succeeded above, so the flag
  // in the return value should always be true.
  DRAKE_DEMAND(value_pair.second == true);
  return value_pair.first;
}

std::optional<double> ReadDoubleFromSdfElement(
    const SDFormatDiagnostic& diagnostic,
    const sdf::ElementConstPtr& parent_element,
    const char* child_element_name) {
  if (parent_element->FindElement(child_element_name) != nullptr) {
    return GetChildElementValue<double>(diagnostic, parent_element,
                                        child_element_name);
  }
  return std::nullopt;
}

}  // namespace

std::unique_ptr<geometry::Shape> MakeShapeFromSdfGeometry(
    const SDFormatDiagnostic& diagnostic, const sdf::Geometry& sdf_geometry,
    ResolveFilename resolve_filename) {
  // TODO(amcastro-tri): unit tests for different error paths are needed.
  // clang-format off
  const std::set<std::string> supported_geometry_elements{
      "box",
      "capsule",
      "cylinder",
      "drake:capsule",
      "drake:ellipsoid",
      "ellipsoid",
      "empty",
      "heightmap",
      "mesh",
      "plane",
      "polyline",
      "sphere"};
  // clang-format on
  CheckSupportedElements(diagnostic, sdf_geometry.Element(),
                         supported_geometry_elements);
  // For the geometry elements parsed by sdformat, assume that all elements
  // are supported and that sdformat is checking the schema.

  switch (sdf_geometry.Type()) {
    case sdf::GeometryType::EMPTY: {
      // TODO(azeey): We should deprecate use of <drake:capsule> and
      // <drake:ellipsoid> per
      // https://github.com/RobotLocomotion/drake/issues/14837

      // Check for custom geometry tags, e.g. drake:capsule.
      if (sdf_geometry.Element()->HasElement("drake:capsule")) {
        const sdf::ElementPtr capsule_element =
            sdf_geometry.Element()->GetElement("drake:capsule");
        CheckSupportedElements(diagnostic, capsule_element,
                               {"radius", "length"});
        std::optional<const double> radius =
            GetChildElementValue<double>(diagnostic, capsule_element, "radius");
        if (!radius.has_value()) return nullptr;
        std::optional<const double> length =
            GetChildElementValue<double>(diagnostic, capsule_element, "length");
        if (!length.has_value()) return nullptr;
        return make_unique<geometry::Capsule>(*radius, *length);
      } else if (sdf_geometry.Element()->HasElement("drake:ellipsoid")) {
        const sdf::ElementPtr ellipsoid_element =
            sdf_geometry.Element()->GetElement("drake:ellipsoid");
        CheckSupportedElements(diagnostic, ellipsoid_element, {"a", "b", "c"});
        std::optional<const double> a =
            GetChildElementValue<double>(diagnostic, ellipsoid_element, "a");
        if (!a.has_value()) return nullptr;
        std::optional<const double> b =
            GetChildElementValue<double>(diagnostic, ellipsoid_element, "b");
        if (!b.has_value()) return nullptr;
        std::optional<const double> c =
            GetChildElementValue<double>(diagnostic, ellipsoid_element, "c");
        if (!c.has_value()) return nullptr;
        return make_unique<geometry::Ellipsoid>(*a, *b, *c);
      }

      return nullptr;
    }
    case sdf::GeometryType::BOX: {
      const sdf::Box& shape = *sdf_geometry.BoxShape();
      const Vector3d box_size = ToVector3(shape.Size());
      return make_unique<geometry::Box>(box_size(0), box_size(1), box_size(2));
    }
    case sdf::GeometryType::CYLINDER: {
      // TODO(amcastro-tri): Verify with @nkoenig that sdf::Cylinder's axis
      // point in the positive z direction as Drake's cylinders do.
      const sdf::Cylinder& shape = *sdf_geometry.CylinderShape();
      return make_unique<geometry::Cylinder>(shape.Radius(), shape.Length());
    }
    case sdf::GeometryType::PLANE: {
      // While sdf::Plane contains the normal of the plane, geometry::HalfSpace
      // only encodes a half space with normal along the z-axis direction of a
      // canonical frame C. Therefore the normal information is used during
      // the parsing of a GeometryInstance, which does contain the pose of the
      // half space in the parent link frame.
      return make_unique<geometry::HalfSpace>();
    }
    case sdf::GeometryType::SPHERE: {
      const sdf::Sphere& shape = *sdf_geometry.SphereShape();
      return make_unique<geometry::Sphere>(shape.Radius());
    }
    case sdf::GeometryType::MESH: {
      // TODO(jwnimmer-tri) Port this to the sdf::Mesh APIs.
      const sdf::ElementConstPtr geometry_element = sdf_geometry.Element();
      DRAKE_DEMAND(geometry_element != nullptr);
      const sdf::ElementConstPtr mesh_element =
          geometry_element->FindElement("mesh");
      DRAKE_DEMAND(mesh_element != nullptr);
      std::optional<std::string> mesh_uri =
          GetChildElementValue<std::string>(diagnostic, mesh_element, "uri");
      // We don't register an error; this is a required element and the sdformat
      // parsing should handle the case of this missing.
      DRAKE_DEMAND(mesh_uri.has_value());
      if (!mesh_uri.has_value()) return nullptr;
      // Note: if the sdf hasn't specified a URI for the <mesh> tag, SDFormat
      // provides "__default__".
      if (*mesh_uri == "__default__") {
        // Note: this is tested in detail_sdf_parser_test.cc in the
        // AutoInertiaForMeshBadData test.
        std::optional<int> line_number_maybe = mesh_element->LineNumber();
        diagnostic.Error(
            mesh_element,
            fmt::format(
                "The <mesh> tag{} is missing the required 'uri' attribute.",
                line_number_maybe
                    ? fmt::format(" on line {}", *line_number_maybe)
                    : std::string()));
        return nullptr;
      }
      const std::string file_name = resolve_filename(diagnostic, *mesh_uri);
      Vector3d scale(1, 1, 1);
      if (mesh_element->HasElement("scale")) {
        std::optional<gz::math::Vector3d> gz_scale =
            GetChildElementValue<gz::math::Vector3d>(diagnostic, mesh_element,
                                                     "scale");
        if (!gz_scale.has_value()) return nullptr;
        scale = Vector3d(gz_scale->X(), gz_scale->Y(), gz_scale->Z());
      }
      // TODO(amcastro-tri): Fix the given path to be an absolute path.
      if (mesh_element->HasElement("drake:declare_convex")) {
        return make_unique<geometry::Convex>(file_name, scale);
      } else {
        return make_unique<geometry::Mesh>(file_name, scale);
      }
    }
    case sdf::GeometryType::CAPSULE: {
      const sdf::Capsule& shape = *sdf_geometry.CapsuleShape();
      return make_unique<geometry::Capsule>(shape.Radius(), shape.Length());
    }
    case sdf::GeometryType::ELLIPSOID: {
      const gz::math::Vector3d& radii = sdf_geometry.EllipsoidShape()->Radii();
      return make_unique<geometry::Ellipsoid>(radii.X(), radii.Y(), radii.Z());
    }
    case sdf::GeometryType::HEIGHTMAP: {
      return std::unique_ptr<geometry::Shape>(nullptr);
    }
    case sdf::GeometryType::POLYLINE: {
      return std::unique_ptr<geometry::Shape>(nullptr);
    }
    case sdf::GeometryType::CONE: {
      return std::unique_ptr<geometry::Shape>(nullptr);
    }
  }

  DRAKE_UNREACHABLE();
}

static constexpr char kAcceptingTag[] = "drake:accepting_renderer";

std::unique_ptr<GeometryInstance> MakeGeometryInstanceFromSdfVisual(
    const SDFormatDiagnostic& diagnostic, const sdf::Visual& sdf_visual,
    ResolveFilename resolve_filename, const math::RigidTransformd& X_LG) {
  // clang-format off
  const std::set<std::string> supported_visual_elements{
      "drake:perception_properties",
      "drake:illustration_properties",
      "geometry",
      "material",
      "pose",
      kAcceptingTag};
  // clang-format on
  CheckSupportedElements(diagnostic, sdf_visual.Element(),
                         supported_visual_elements);

  const sdf::Geometry& sdf_geometry = *sdf_visual.Geom();
  if (sdf_geometry.Type() == sdf::GeometryType::EMPTY) {
    // The file either specifies an EMPTY geometry or one that isn't recognized
    // by libsdf. We first check for any custom geometry tags, e.g.
    // drake:capsule, before we can decide to return a null geometry.
    if (!sdf_geometry.Element()->HasElement("drake:capsule") &&
        !sdf_geometry.Element()->HasElement("drake:ellipsoid")) {
      return nullptr;
    }
  }

  // GeometryInstance defines its shapes in a "canonical frame" C. For instance:
  // - A half-space's normal is directed along the Cz axis,
  // - A cylinder's length is parallel to the Cz axis,
  // - etc.

  // X_LC defines the pose of the canonical frame in the link frame L.
  // N.B. In most cases C coincides with the SDF G frame.
  RigidTransformd X_LC = X_LG;

  // For a half-space, C and G are not the same since SDF allows to specify
  // the normal of the plane in the G frame.
  // Note to developers: if needed, update this switch statement to consider
  // other geometry types whenever X_LC != X_LG.
  switch (sdf_geometry.Type()) {
    case sdf::GeometryType::EMPTY:  // Also includes custom geometries.
    case sdf::GeometryType::HEIGHTMAP:
    case sdf::GeometryType::BOX:
    case sdf::GeometryType::CAPSULE:
    case sdf::GeometryType::CONE:
    case sdf::GeometryType::CYLINDER:
    case sdf::GeometryType::ELLIPSOID:
    case sdf::GeometryType::MESH:
    case sdf::GeometryType::POLYLINE:
    case sdf::GeometryType::SPHERE: {
      // X_LC = X_LG for these geometries.
      break;
    }
    case sdf::GeometryType::PLANE: {
      const sdf::Plane& shape = *sdf_geometry.PlaneShape();
      // TODO(amcastro-tri): we assume the normal is in the frame of the visual
      // geometry G. Verify this with @nkoenig.
      const Vector3d normal_G = ToVector3(shape.Normal());
      // sdf::Plane also has sdf::Plane::Size(), but we ignore it since in Drake
      // planes are entire half-spaces.

      // The normal expressed in the frame G defines the pose of the half space
      // in its canonical frame C in which the normal aligns with the z-axis
      // direction.
      const RigidTransformd X_GC(
          geometry::HalfSpace::MakePose(normal_G, Vector3d::Zero()));

      // Correct X_LC to include the pose X_GC
      X_LC = X_LG * X_GC;
      break;
    }
  }

  std::unique_ptr<geometry::Shape> shape =
      MakeShapeFromSdfGeometry(diagnostic, sdf_geometry, resolve_filename);
  if (shape == nullptr) return nullptr;

  auto instance =
      make_unique<GeometryInstance>(X_LC, std::move(*shape), sdf_visual.Name());
  VisualProperties vis_props = MakeVisualPropertiesFromSdfVisual(
      diagnostic, sdf_visual, resolve_filename);
  if (!(vis_props.illustration.has_value() ||
        vis_props.perception.has_value())) {
    // No properties of any kind were returned.
    return nullptr;
  }
  if (vis_props.illustration.has_value()) {
    instance->set_illustration_properties(*std::move(vis_props.illustration));
  }
  if (vis_props.perception.has_value()) {
    instance->set_perception_properties(*std::move(vis_props.perception));
  }
  return instance;
}

namespace {

// The element is only disabled iff:
//  - it is defined and
//  - it has the attribute "enabled" and
//  - the attribute's value is false.
// In all other cases, the element is considered enabled.
bool ElementIsEnabled(const sdf::ElementConstPtr e,
                      const SDFormatDiagnostic& diagnostic) {
  if (e == nullptr) {
    return true;
  }
  sdf::ParamPtr attr = e->GetAttribute("enabled");
  if (attr == nullptr) {
    return true;
  }
  bool enabled;
  const bool valid = attr->Get(enabled);
  if (!valid) {
    diagnostic.Error(
        e,
        fmt::format(
            "The `{}` element has an 'enabled' attribute with the wrong value "
            "type; it should be a boolean value.",
            e->GetName()));
    return false;
  }
  return enabled;
}

}  // namespace

VisualProperties MakeVisualPropertiesFromSdfVisual(
    const SDFormatDiagnostic& diagnostic, const sdf::Visual& sdf_visual,
    ResolveFilename resolve_filename) {
  // This doesn't directly use the sdf::Material API on purpose. In the current
  // version, if a parameter (e.g., diffuse) is missing it will *not* be
  // included in the geometry properties. Using the sdf::Material, it is
  // impossible to tell if this is happening. If the material exists, then
  // diffuse, ambient, etc., all have default values and those values will be
  // written to the geometry properties. This breaks the ability of the
  // downstream consumer to supply its own defaults (because it can't
  // distinguish between a value that was specified by the user and one that was
  // provided by sdformat's default value).

  const sdf::ElementPtr visual_element = sdf_visual.Element();
  // TODO(SeanCurtis-TRI): These elements will be more fully utilized when these
  // tags contain child tags that configure properties.
  const sdf::ElementConstPtr percep =
      visual_element->FindElement("drake:perception_properties");
  const sdf::ElementConstPtr illus =
      visual_element->FindElement("drake:illustration_properties");
  const bool gets_percep = ElementIsEnabled(percep, diagnostic);
  const bool gets_illus = ElementIsEnabled(illus, diagnostic);

  if (!(gets_percep || gets_illus)) {
    // The <visual> tag has had both illustration and perception roles disabled.

    // If true, this visual was a <visual> and not a <drake:visual>.
    const bool is_sdf_visual =
        sdf_visual.Element()->GetAttribute(kIsDrakeNamespaceAttr) == nullptr;
    if (is_sdf_visual) {
      diagnostic.Warning(
          visual_element,
          fmt::format("The <visual name=\"{}\"> tag had all visual roles "
                      "turned off for Drake; this model may appear very "
                      "different in Drake from other sdformat loaders.",
                      sdf_visual.Name()));
    }

    return {std::nullopt, std::nullopt};
  }

  // At the point, we know at least one visual role has been specified. If
  // illustration is defined, we'll set the properties on illustration
  // properties and (maybe) copy them over to perception when we're done.
  // Otherwise, we write directly to the perception_properties.
  std::optional<IllustrationProperties> illus_props;
  std::optional<PerceptionProperties> percep_props;
  GeometryProperties* properties = nullptr;
  if (gets_illus) {
    illus_props = IllustrationProperties();
    properties = &*illus_props;
  } else {
    percep_props = PerceptionProperties();
    properties = &*percep_props;
  }

  // Element pointers can only be nullptr if Load() was not called on the sdf::
  // object. Only a bug could cause this.
  DRAKE_DEMAND(visual_element != nullptr);

  const sdf::ElementConstPtr material_element =
      visual_element->FindElement("material");

  if (material_element.get() != nullptr) {
    // clang-format off
    const std::set<std::string> supported_material_elements{
        "ambient",
        "diffuse",
        "drake:diffuse_map",
        "emissive",
        "specular"};
    // clang-format on
    CheckSupportedElements(diagnostic, material_element,
                           supported_material_elements);

    if (material_element->HasElement("drake:diffuse_map")) {
      auto [texture_name, has_value] =
          material_element->Get<std::string>("drake:diffuse_map", {});
      if (has_value) {
        const std::string resolved_path =
            resolve_filename(diagnostic, texture_name);
        if (resolved_path.empty()) {
          std::string message = std::string(fmt::format(
              "Unable to locate the texture file: {}", texture_name));
          diagnostic.Error(visual_element, std::move(message));
          return {std::nullopt, std::nullopt};
        }
        properties->AddProperty("phong", "diffuse_map", resolved_path);
      }
    }

    auto add_property = [material_element](const char* property,
                                           GeometryProperties* props) {
      if (!material_element->HasElement(property)) return;
      using gz::math::Color;
      const std::pair<Color, bool> value_pair =
          material_element->Get<Color>(property, Color());
      if (value_pair.second == false) return;
      const Color& sdf_color = value_pair.first;

      Vector4<double> color{sdf_color.R(), sdf_color.G(), sdf_color.B(),
                            sdf_color.A()};
      props->AddProperty("phong", property, color);
    };

    add_property("diffuse", properties);
    add_property("ambient", properties);
    add_property("specular", properties);
    add_property("emissive", properties);
  }

  if (gets_percep && gets_illus) {
    percep_props = PerceptionProperties(*illus_props);
  }

  // TODO(SeanCurtis-TRI): As we allow customization of properties within the
  // drake::foo_properties tags, refine the illustration and perception
  // properties as appropriate.

  // Perception-only properties.
  if (visual_element->HasElement(kAcceptingTag)) {
    sdf::ElementPtr accepting = visual_element->GetElement(kAcceptingTag);
    if (gets_percep) {
      set<string> accepting_names;
      while (accepting != nullptr) {
        const string& name = accepting->Get<string>();
        if (name.empty()) {
          std::string message =
              fmt::format("<{}> tag given without any name", kAcceptingTag);
          diagnostic.Error(accepting, std::move(message));
          return {std::nullopt, std::nullopt};
        }
        accepting_names.insert(name);
        accepting = accepting->GetNextElement(kAcceptingTag);
      }
      DRAKE_DEMAND(accepting_names.size() > 0);
      percep_props->AddProperty("renderer", "accepting",
                                std::move(accepting_names));
    } else {
      diagnostic.Warning(
          visual_element,
          fmt::format(
              "<{}> specified for a <visual> with a disabled perception role.",
              kAcceptingTag));
    }
  }

  return {illus_props, percep_props};
}

RigidTransformd MakeGeometryPoseFromSdfCollision(
    const sdf::Collision& sdf_collision, const RigidTransformd& X_LG) {
  // GeometryInstance defines its shapes in a "canonical frame" C. The canonical
  // frame C is the frame in which the geometry is defined and it generally
  // coincides with the geometry frame G (G is specified in the SDF file).
  // For instance:
  // - A half-space's normal is directed along the Cz axis,
  // - A cylinder's length is parallel to the Cz axis,
  // - etc.
  // There are cases however in which C might not coincide with G. A HalfSpace
  // is one of such examples, since for geometry::HalfSpace the normal is
  // represented in the C frame along Cz, whereas SDF defines the normal in a
  // frame G which does not necessarily coincide with C.

  // X_LC defines the pose of the canonical frame in the link frame L.
  // N.B. In most cases C coincides with the SDF G frame.
  RigidTransformd X_LC = X_LG;

  // For a half-space, C and G are not the same since SDF allows to specify
  // the normal of the plane in the G frame.
  // Note to developers: if needed, update this switch statement to consider
  // other geometry types whenever X_LC != X_LG.
  const sdf::Geometry& sdf_geometry = *sdf_collision.Geom();
  switch (sdf_geometry.Type()) {
    case sdf::GeometryType::EMPTY:
    case sdf::GeometryType::HEIGHTMAP:
    case sdf::GeometryType::BOX:
    case sdf::GeometryType::CAPSULE:
    case sdf::GeometryType::CONE:
    case sdf::GeometryType::CYLINDER:
    case sdf::GeometryType::ELLIPSOID:
    case sdf::GeometryType::MESH:
    case sdf::GeometryType::POLYLINE:
    case sdf::GeometryType::SPHERE: {
      // X_LC = X_LG for these geometries.
      break;
    }
    case sdf::GeometryType::PLANE: {
      const sdf::Plane& shape = *sdf_geometry.PlaneShape();
      const Vector3d normal_G = ToVector3(shape.Normal());
      // sdf::Plane also has sdf::Plane::Size(), but we ignore it since in Drake
      // planes are entire half-spaces.

      // The normal expressed in the frame G defines the pose of the half space
      // in its canonical frame C in which the normal aligns with the z-axis
      // direction.
      const RigidTransformd X_GC(
          geometry::HalfSpace::MakePose(normal_G, Vector3d::Zero()));

      // Correct X_LC to include the pose X_GC
      X_LC = X_LG * X_GC;
      break;
    }
  }
  return X_LC;
}

std::optional<ProximityProperties> MakeProximityPropertiesForCollision(
    const SDFormatDiagnostic& diagnostic, const sdf::Collision& sdf_collision) {
  const sdf::ElementPtr collision_element = sdf_collision.Element();
  DRAKE_DEMAND(collision_element != nullptr);

  // clang-format off
  const std::set<std::string> supported_collision_elements{
      "drake:proximity_properties",
      "geometry",
      "laser_retro",
      "pose",
      "surface"};
  // clang-format on
  CheckSupportedElements(diagnostic, collision_element,
                         supported_collision_elements);
  CheckSupportedElementValue(diagnostic, collision_element, "laser_retro", "0");

  const sdf::ElementConstPtr drake_element =
      collision_element->FindElement("drake:proximity_properties");

  geometry::ProximityProperties properties;
  if (drake_element != nullptr) {
    // clang-format off
    const std::set<std::string> supported_proximity_elements{
        "drake:rigid_hydroelastic",
        "drake:compliant_hydroelastic",
        "drake:soft_hydroelastic",
        "drake:mesh_resolution_hint",
        "drake:hydroelastic_modulus",
        "drake:hydroelastic_margin",
        "drake:hunt_crossley_dissipation",
        "drake:relaxation_time",
        "drake:point_contact_stiffness",
        "drake:mu_dynamic",
        "drake:mu_static"};
    // clang-format on
    CheckSupportedElements(diagnostic, drake_element,
                           supported_proximity_elements);

    auto read_double = [&diagnostic, &drake_element](const char* element_name) {
      return ReadDoubleFromSdfElement(diagnostic, drake_element, element_name);
    };

    const bool is_rigid = drake_element->HasElement("drake:rigid_hydroelastic");
    const bool is_compliant =
        drake_element->HasElement("drake:compliant_hydroelastic");

    // TODO(16229): Remove this ad-hoc input sanitization when we resolve
    //  issue 16229 "Diagnostics for unsupported SDFormat and URDF stanzas."
    const bool is_unsupported_soft =
        drake_element->HasElement("drake:soft_hydroelastic");
    if (is_unsupported_soft) {
      std::string message =
          "A <collision> geometry has defined the "
          "unsupported tag <drake:soft_hydroelastic>. Please change it to "
          "<drake:compliant_hydroelastic>.";
      diagnostic.Error(collision_element, std::move(message));
      return std::nullopt;
    }

    if (is_rigid && is_compliant) {
      std::string message =
          "A <collision> geometry has defined "
          "mutually-exclusive tags <drake:rigid_hydroelastic> and "
          "<drake:compliant_hydroelastic>. Only one can be provided.";
      diagnostic.Error(collision_element, std::move(message));
      return std::nullopt;
    }

    properties =
        ParseProximityProperties(diagnostic.MakePolicyForNode(*drake_element),
                                 read_double, is_rigid, is_compliant);
  }

  // TODO(SeanCurtis-TRI): Remove all of this legacy parsing code based on
  //  issue #12598.
  if (!properties.HasProperty(geometry::internal::kMaterialGroup,
                              geometry::internal::kFriction)) {
    std::optional<CoulombFriction<double>> coulomb_friction =
        MakeCoulombFrictionFromSdfCollisionOde(diagnostic, sdf_collision);
    if (!coulomb_friction.has_value()) return std::nullopt;
    properties.AddProperty(geometry::internal::kMaterialGroup,
                           geometry::internal::kFriction, *coulomb_friction);
  } else {
    // We parsed friction from <drake:proximity_properties>; test for the
    // existence of the legacy mechanism and warn we're not using it.
    const sdf::ElementConstPtr surface_element =
        collision_element->FindElement("surface");
    if (surface_element.get()) {
      CheckSupportedElements(diagnostic, surface_element, {"friction"});
      const sdf::ElementConstPtr friction_element =
          surface_element->FindElement("friction");
      if (friction_element.get()) {
        CheckSupportedElements(diagnostic, friction_element, {"ode"});
        const sdf::ElementConstPtr ode_element =
            friction_element->FindElement("ode");
        CheckSupportedElements(diagnostic, ode_element, {"mu", "mu2"});
        if (ode_element->FindElement("mu").get() ||
            ode_element->FindElement("mu2").get()) {
          diagnostic.Warning(
              ode_element,
              fmt::format(
                  "In <collision name='{}'>: "
                  "When drake contact parameters are fully specified in the "
                  "<drake:proximity_properties> tag, the "
                  "<surface><friction><ode>"
                  "<mu*> tags are ignored.",
                  sdf_collision.Name()));
        }
      }
    }
  }

  return properties;
}

std::optional<CoulombFriction<double>> MakeCoulombFrictionFromSdfCollisionOde(
    const SDFormatDiagnostic& diagnostic, const sdf::Collision& sdf_collision) {
  const sdf::ElementPtr collision_element = sdf_collision.Element();
  // Element pointers can only be nullptr if Load() was not called on the sdf::
  // object. Only a bug could cause this.
  DRAKE_DEMAND(collision_element != nullptr);

  // Look for a surface/friction/ode element. If any are missing, we return
  // default friction properties.
  // TODO(eric.cousineau): Use sdf::Surface once it is more complete.
  const sdf::ElementConstPtr surface_element =
      collision_element->FindElement("surface");
  if (!surface_element.get()) return default_friction();
  const sdf::ElementConstPtr friction_element =
      surface_element->FindElement("friction");
  if (!friction_element.get()) return default_friction();
  const sdf::ElementConstPtr ode_element = friction_element->FindElement("ode");
  if (!ode_element.get()) return default_friction();

  // Read <mu> (for static) and <mu2> (for dynamic), with default values.
  std::optional<const double> static_friction = GetChildElementValue<double>(
      diagnostic, ode_element, "mu", default_friction().static_friction());
  if (!static_friction.has_value()) return std::nullopt;
  std::optional<const double> dynamic_friction = GetChildElementValue<double>(
      diagnostic, ode_element, "mu2", default_friction().dynamic_friction());
  if (!dynamic_friction.has_value()) return std::nullopt;

  return CoulombFriction<double>(*static_friction, *dynamic_friction);
}

std::optional<geometry::ProximityProperties>
MakeProximityForDeformableCollision(const SDFormatDiagnostic& diagnostic,
                                    const sdf::Collision& collision) {
  // Allowed child tags of <collision> for our mini‑parser.
  sdf::ElementPtr collision_element = collision.Element();
  CheckSupportedElements(diagnostic, collision_element,
                         {"geometry", "drake:proximity_properties"});

  const sdf::ElementPtr drake_element =
      collision_element->FindElement("drake:proximity_properties");
  if (!drake_element) return std::nullopt;  // no properties specified.

  CheckSupportedElements(diagnostic, drake_element,
                         {"drake:mu_dynamic", "drake:hunt_crossley_dissipation",
                          "drake:relaxation_time"});

  geometry::ProximityProperties props;

  auto read_double = [&diagnostic, &drake_element](const char* element_name) {
    return ReadDoubleFromSdfElement(diagnostic, drake_element, element_name);
  };

  const std::optional<double> mu_dynamic = read_double("drake:mu_dynamic");
  if (mu_dynamic.has_value()) {
    if (*mu_dynamic < 0) {
      diagnostic.Error(drake_element, "drake:mu_dynamic must be non‑negative");
      return std::nullopt;
    } else {
      props.AddProperty("material", "coulomb_friction",
                        CoulombFriction<double>(*mu_dynamic, *mu_dynamic));
    }
  } else {
    // If no value is specified, we use the default value.
    props.AddProperty("material", "coulomb_friction", default_friction());
  }

  const std::optional<double> hunt_crossley_dissipation =
      read_double("drake:hunt_crossley_dissipation");
  if (hunt_crossley_dissipation.has_value()) {
    if (*hunt_crossley_dissipation < 0) {
      diagnostic.Error(drake_element,
                       "drake:hunt_crossley_dissipation must be non‑negative");
      return std::nullopt;
    }
    props.AddProperty("material", "hunt_crossley_dissipation",
                      *hunt_crossley_dissipation);
  }

  const std::optional<double> relaxation_time =
      read_double("drake:relaxation_time");
  if (relaxation_time.has_value()) {
    if (*relaxation_time < 0) {
      diagnostic.Error(drake_element,
                       "drake:relaxation_time must be non‑negative");
      return std::nullopt;
    }
    props.AddProperty("material", "relaxation_time", *relaxation_time);
  }
  return props;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
