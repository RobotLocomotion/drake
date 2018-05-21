#include "drake/multibody/multibody_tree/parsing/scene_graph_parser_detail.h"

#include <memory>
#include <string>
#include <utility>

#include <sdf/sdf.hh>

#include "drake/geometry/geometry_instance.h"
#include "drake/multibody/multibody_tree/parsing/sdf_parser_common.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace detail {

using Eigen::Isometry3d;
using Eigen::Vector3d;
using geometry::GeometryInstance;
using std::make_unique;

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
    // signature.
    return const_cast<sdf::Element&>(element).GetElement(child_name).get();
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

  // We deal with the <mesh> case separately since sdf::Geometry still does not
  // support it.
  // TODO(amcastro-tri): get rid of all sdf::ElementPtr once
  // sdf::GeometryType::MESH is available.
  const sdf::Element* const geometry_element = sdf_geometry.Element().get();
  DRAKE_DEMAND(geometry_element != nullptr);
  const sdf::Element* const mesh_element =
      MaybeGetChildElement(*geometry_element, "mesh");
  if (mesh_element != nullptr) {
    const std::string file_name =
        GetChildElementValueOrThrow<std::string>(*mesh_element, "uri");
    double scale = 1.0;
    if (mesh_element->HasElement("scale")) {
      const ignition::math::Vector3d& scale_vector =
          GetChildElementValueOrThrow<ignition::math::Vector3d>(
              *mesh_element, "scale");
      // geometry::Mesh only supports isotropic scaling and therefore we enforce
      // it.
      if (!(scale_vector.X() == scale_vector.Y() &&
            scale_vector.X() == scale_vector.Z())) {
        throw std::runtime_error(
            "Drake meshes only support isotropic scaling. Therefore all three "
                "scaling factors must be exactly equal.");
      }
      scale = scale_vector.X();
    }
    // TODO(amcastro-tri): Fix the given path to be an absolute path.
    return make_unique<geometry::Mesh>(file_name, scale);
  }

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
    // TODO(amcastro-tri): When sdformat supports it add the MESH case.
    default: {
      throw std::logic_error("Geometry type not supported.");
    }
  }
}

std::unique_ptr<GeometryInstance> MakeGeometryInstanceFromSdfVisual(
    const sdf::Visual& sdf_visual) {
  // Retrieve the pose of the visual frame G in the parent link L in which
  // geometry gets defined.
  const Isometry3d X_LG = ToIsometry3(sdf_visual.Pose());
  const sdf::Geometry& sdf_geometry = *sdf_visual.Geom();

  // GeometryInstance defines its shapes in a "canonical frame" C. For instance:
  // - A half-space's normal is directed along the Cz axis,
  // - A cylinder's length is parallel to the Cz axis,
  // - etc.

  // X_LC defines the pose of the canonical frame in the link frame L.
  Isometry3d X_LC = X_LG;  // In most cases C coincides with the SDF G frame.

  // We deal with the <mesh> case separately since sdf::Geometry still does not
  // support it and marks <mesh> geometry with type sdf::GeometryType::EMPTY.
  // Therefore, there are two reasons we can have an EMPTY type:
  //   1) The file does specify a mesh.
  //   2) The file truly specifies an EMPTY geometry.
  // We treat these two cases separately until sdformat provides support for
  // meshes.
  // TODO(amcastro-tri): Cleanup usage of sdf::ElementPtr once sdformat gets
  // extended to support more data representation types.
  if (sdf_geometry.Type() == sdf::GeometryType::EMPTY) {
    sdf::ElementPtr geometry_element = sdf_geometry.Element();
    DRAKE_DEMAND(geometry_element != nullptr);
    // Case 1: We do have a mesh.
    if (geometry_element->HasElement("mesh")) {
      return make_unique<GeometryInstance>(
          X_LC, MakeShapeFromSdfGeometry(sdf_geometry));
    } else {
      // Case 2: The file truly specifies an EMPTY geometry.
      return std::unique_ptr<GeometryInstance>(nullptr);
    }
  }

  // For a half-space, C and G are not the same since  SDF allows to specify
  // the normal of the plane in the G frame.
  if (sdf_geometry.Type() == sdf::GeometryType::PLANE) {
    const sdf::Plane& shape = *sdf_geometry.PlaneShape();
    // TODO(amcastro-tri): we assume the normal is in the frame of the visual
    // geometry G. Verify this with @nkoenig.
    const Vector3d normal_G = ToVector3(shape.Normal());
    // sdf::Plane also has sdf::Plane::Size(), but we ignore it since in Drake
    // planes are entire half-spaces.

    // The normal expressed in the frame G defines the pose of the half space
    // in its canonical frame C in which the normal aligns with the z-axis
    // direction.
    const Isometry3d X_GC =
        geometry::HalfSpace::MakePose(normal_G, Vector3d::Zero());

    // Correct X_LC to include the pose X_GC
    X_LC = X_LG * X_GC;
  }

  // TODO(amcastro-tri): Extract <material> once sdf::Visual supports it.

  return make_unique<GeometryInstance>(
      X_LC, MakeShapeFromSdfGeometry(sdf_geometry));
}

}  // namespace detail

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
