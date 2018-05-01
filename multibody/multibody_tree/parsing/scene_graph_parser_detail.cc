#include "drake/multibody/multibody_tree/parsing/scene_graph_parser_detail.h"

#include <memory>

#include <sdf/sdf.hh>

#include "drake/multibody/multibody_tree/parsing/sdf_parser_common.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace detail {

using Eigen::Vector3d;
using std::make_unique;

std::unique_ptr<geometry::Shape> MakeShapeFromSdfGeometry(
    const sdf::Geometry& sdf_geometry) {
  switch (sdf_geometry.Type()) {
    case sdf::GeometryType::CYLINDER: {
      const sdf::Cylinder& shape = *sdf_geometry.CylinderShape();
      return make_unique<geometry::Cylinder>(shape.Radius(), shape.Length());
    }
    case sdf::GeometryType::SPHERE: {
      const sdf::Sphere& shape = *sdf_geometry.SphereShape();
      return make_unique<geometry::Sphere>(shape.Radius());
    }
    case sdf::GeometryType::PLANE: {
      //const sdf::Plane& shape = *sdf_geometry.PlaneShape();
      return make_unique<geometry::HalfSpace>();

      // TODO(amcastro-tri): We assume the normal is in the model frame M. We
      // need to verify this is an SDF specification invariant.
      //const Vector3d normal_M = ToVector3(shape.Normal());
          //make_unique<geometry::GeometryInstance>(
          //geometry::HalfSpace::MakePose(normal_M, Vector3d::Zero(),
            //                               make_unique<HalfSpace>()));
    }
    default: {
      throw std::logic_error("Geometry type not supported.");
    }
  }
}

}  // namespace detail

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
