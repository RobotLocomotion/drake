#include "drake/multibody/parsing/detail_scene_graph.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <sdf/sdf.hh>

#include "drake/geometry/geometry_instance.h"
#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_ignition.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/plant/coulomb_friction.h"

namespace drake {
namespace multibody {
namespace internal {

using Eigen::Vector3d;
using std::make_unique;

using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using math::RigidTransformd;

namespace {

// Helper to return the child element of `element` named `child_name`.
// Returns nullptr if not present.
const sdf::Element* MaybeGetChildElement(
    const sdf::Element& element, const std::string &child_name) {
  // First verify <child_name> is present (otherwise GetElement() has the
  // side effect of adding new elements if not present!!).
  if (element.HasElement(child_name)) {
    // NOTE: The const_cast() here is needed because sdformat does not provide
    // a const version of GetElement(). However, the snippet below still
    // guarantees "element" is not changed as promised by this method's
    // signature. See sdformat issue #188.
    return const_cast<sdf::Element&>(element).GetElement(child_name).get();
  }
  return nullptr;
}

// Helper to return the child element of `element` named `child_name`.
// Throws std::runtime_error if not found.
const sdf::Element& GetChildElementOrThrow(
    const sdf::Element& element, const std::string &child_name) {
  // First verify <child_name> is present (otherwise GetElement() has the
  // side effect of adding new elements if not present!!).
  if (!element.HasElement(child_name)) {
    throw std::runtime_error(
        "Element <" + child_name + "> not found nested within element <" +
            element.GetName() + ">.");
  }
  // NOTE: The const_cast() here is needed because sdformat does not provide
  // a const version of GetElement(). However, the snippet below still
  // guarantees "element" is not changed as promised by this method's
  // signature. See sdformat issue #188.
  return *const_cast<sdf::Element &>(element).GetElement(child_name);
}

// Helper to return the mutable child element of `element` named
// `child_name`.  Returns nullptr if not present.
sdf::Element* MaybeGetChildElement(
    sdf::Element* element, const std::string &child_name) {
  // First verify <child_name> is present (otherwise GetElement() has the
  // side effect of adding new elements if not present!!).
  if (element->HasElement(child_name)) {
    return element->GetElement(child_name).get();
  }
  return nullptr;
}

// Helper to return the value of a child of `element` named `child_name`.
// A std::runtime_error is thrown if the `<child_name>` tag is missing from the
// SDF file, or the tag has a bad or missing value.
template <typename T>
T GetChildElementValueOrThrow(const sdf::Element& element,
                              const std::string& child_name) {
  // TODO(amcastro-tri): unit tests for different error paths are needed.
  if (!element.HasElement(child_name)) {
    throw std::runtime_error(
        "Element <" + child_name + "> is required within element "
            "<" + element.GetName() + ">.");
  }
  std::pair<T, bool> value_pair = element.Get<T>(child_name, T());
  if (value_pair.second == false) {
    // TODO(amcastro-tri): Figure out a way to throw meaningful error messages
    // with line/row numbers within the file.
    throw std::runtime_error(
        "Invalid value for <" + child_name + "> within element "
            "<" + element.GetName() + ">.");
  }
  return value_pair.first;
}

}  // namespace

std::unique_ptr<geometry::Shape> MakeShapeFromSdfGeometry(
    const sdf::Geometry& sdf_geometry) {
  // TODO(amcastro-tri): unit tests for different error paths are needed.

  switch (sdf_geometry.Type()) {
    case sdf::GeometryType::EMPTY: {
      return std::unique_ptr<geometry::Shape>(nullptr);
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
      const sdf::Element* const geometry_element = sdf_geometry.Element().get();
      DRAKE_DEMAND(geometry_element != nullptr);
      const sdf::Element* const mesh_element =
          MaybeGetChildElement(*geometry_element, "mesh");
      DRAKE_DEMAND(mesh_element != nullptr);
      const std::string file_name =
          GetChildElementValueOrThrow<std::string>(*mesh_element, "uri");
      double scale = 1.0;
      if (mesh_element->HasElement("scale")) {
        const ignition::math::Vector3d& scale_vector =
            GetChildElementValueOrThrow<ignition::math::Vector3d>(
                *mesh_element, "scale");
        // geometry::Mesh only supports isotropic scaling and therefore we
        // enforce it.
        if (!(scale_vector.X() == scale_vector.Y() &&
              scale_vector.X() == scale_vector.Z())) {
          throw std::runtime_error(
              "Drake meshes only support isotropic scaling. Therefore all "
              "three scaling factors must be exactly equal.");
        }
        scale = scale_vector.X();
      }

      // TODO(amcastro-tri): Fix the given path to be an absolute path.
      if (mesh_element->HasElement("drake:declare_convex")) {
        return make_unique<geometry::Convex>(file_name, scale);
      } else {
        return make_unique<geometry::Mesh>(file_name, scale);
      }
    }
  }

  DRAKE_UNREACHABLE();
}

std::unique_ptr<GeometryInstance> MakeGeometryInstanceFromSdfVisual(
    const sdf::Visual& sdf_visual) {
  const sdf::Geometry& sdf_geometry = *sdf_visual.Geom();
  if (sdf_geometry.Type() == sdf::GeometryType::EMPTY) {
    // The file specifies an EMPTY geometry.
    return std::unique_ptr<GeometryInstance>(nullptr);
  }

  // Retrieve the pose of the visual frame G in the parent link L in which
  // geometry gets defined.
  const RigidTransformd X_LG = ToRigidTransform(sdf_visual.Pose());

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
    case sdf::GeometryType::EMPTY:
    case sdf::GeometryType::BOX:
    case sdf::GeometryType::CYLINDER:
    case sdf::GeometryType::MESH:
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

  auto instance = make_unique<GeometryInstance>(
      X_LC, MakeShapeFromSdfGeometry(sdf_geometry), sdf_visual.Name());
  instance->set_illustration_properties(
      MakeVisualPropertiesFromSdfVisual(sdf_visual));
  return instance;
}

IllustrationProperties MakeVisualPropertiesFromSdfVisual(
    const sdf::Visual& sdf_visual) {
  // TODO(SeanCurtis-TRI): Update this to use the sdf API when
  // https://bitbucket.org/osrf/sdformat/pull-requests/445/material-dom/diff
  // merges.
  // The existence of a visual element will *always* require an
  // IllustrationProperties instance. How we populate it depends on the material
  // values.
  IllustrationProperties properties;

  const sdf::ElementPtr visual_element = sdf_visual.Element();
  // Element pointers can only be nullptr if Load() was not called on the sdf::
  // object. Only a bug could cause this.
  DRAKE_DEMAND(visual_element != nullptr);

  const sdf::Element* const material_element =
      MaybeGetChildElement(*visual_element, "material");

  if (material_element != nullptr) {
    auto add_property = [material_element](
        const char* property, IllustrationProperties* props) {
      if (!material_element->HasElement(property)) return;
      using ignition::math::Color;
      const std::pair<Color, bool> value_pair =
          material_element->Get<Color>(property, Color());
      if (value_pair.second == false) return;
      const Color& sdf_color = value_pair.first;

      Vector4<double> color{sdf_color.R(), sdf_color.G(), sdf_color.B(),
                            sdf_color.A()};
      props->AddProperty("phong", property, color);
    };

    add_property("diffuse", &properties);
    add_property("ambient", &properties);
    add_property("specular", &properties);
    add_property("emissive", &properties);
  }

  return properties;
}

RigidTransformd MakeGeometryPoseFromSdfCollision(
    const sdf::Collision& sdf_collision) {
  // Retrieve the pose of the collision frame G in the parent link L in which
  // geometry gets defined.
  const RigidTransformd X_LG = ToRigidTransform(sdf_collision.Pose());

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
    case sdf::GeometryType::BOX:
    case sdf::GeometryType::CYLINDER:
    case sdf::GeometryType::MESH:
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

CoulombFriction<double> MakeCoulombFrictionFromSdfCollisionOde(
    const sdf::Collision& sdf_collision) {

  const sdf::ElementPtr collision_element = sdf_collision.Element();
  // Element pointers can only be nullptr if Load() was not called on the sdf::
  // object. Only a bug could cause this.
  DRAKE_DEMAND(collision_element != nullptr);

  const sdf::Element* const surface_element =
      MaybeGetChildElement(*collision_element, "surface");

  // If the surface is not found, we return default friction properties.
  if (!surface_element) return default_friction();

  // Once <surface> is found, <friction> and <ode> are required.
  const sdf::Element& friction_element =
      GetChildElementOrThrow(*surface_element, "friction");
  const sdf::Element& ode_element =
      GetChildElementOrThrow(friction_element, "ode");

  // Once <ode> is found, <mu> (for static) and <mu2> (for dynamic) are
  // required.
  const double static_friction =
      GetChildElementValueOrThrow<double>(ode_element, "mu");
  const double dynamic_friction =
      GetChildElementValueOrThrow<double>(ode_element, "mu2");

  return CoulombFriction<double>(static_friction, dynamic_friction);
}

sdf::Visual ResolveVisualUri(const sdf::Visual& original,
                             const PackageMap& package_map,
                             const std::string& root_dir) {
  std::shared_ptr<sdf::Element> visual_element = original.Element()->Clone();
  sdf::Element* geom_element =
      MaybeGetChildElement(visual_element.get(), "geometry");
  if (geom_element) {
    sdf::Element* mesh_element = MaybeGetChildElement(geom_element, "mesh");
    if (mesh_element) {
      sdf::Element* uri_element = MaybeGetChildElement(mesh_element, "uri");
      if (uri_element) {
        const std::string uri = uri_element->Get<std::string>();
        const std::string resolved_name =
            ResolveUri(uri, package_map, root_dir);
        if (!resolved_name.empty()) {
          uri_element->Set(resolved_name);
        } else {
          throw std::runtime_error(
              std::string(__FILE__) + ": " + __func__ +
              ": ERROR: Mesh file name could not be resolved from the "
              "provided uri \"" + uri + "\".");
        }
      } else {
        throw std::runtime_error(
            std::string(__FILE__) + ": " + __func__ +
            ": ERROR: <mesh> tag specified without <uri?>");
      }
    }
  }

  sdf::Visual visual;
  visual.Load(visual_element);
  return visual;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
