#include "drake/multibody/multibody_tree/parsing/scene_graph_parser_detail.h"

#include <memory>

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

std::unique_ptr<geometry::Shape> MakeShapeFromSdfGeometry(
    const sdf::Geometry& sdf_geometry) {
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

  // Nothing left to do, return nullptr signaling this is an empty visual.
  if (sdf_geometry.Type() == sdf::GeometryType::EMPTY) {
    return std::unique_ptr<GeometryInstance>(nullptr);
  }

  // GeometryInstance defines its shapes in a "canonical frame" C. For instance:
  // - A half-space's normal is directed along the Cz axis,
  // - A cylinder's length is parallel to the Cz axis,
  // - etc.

  // X_LC defines the pose of the canonical frame in the link frame L.
  Isometry3d X_LC = X_LG;  // In most cases C coincides with the SDF G frame.

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
