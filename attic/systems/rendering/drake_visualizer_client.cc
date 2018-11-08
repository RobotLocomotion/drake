#include "drake/systems/rendering/drake_visualizer_client.h"

#include "drake/math/rotation_matrix.h"

namespace drake {
namespace systems {
namespace rendering {

lcmt_viewer_geometry_data MakeGeometryData(
    const DrakeShapes::VisualElement& visual_element) {
  lcmt_viewer_geometry_data geometry_data;
  const DrakeShapes::Geometry& geometry = visual_element.getGeometry();

  // TODO(liang.fok) Do this through virtual methods without introducing any
  // LCM dependency on the Geometry classes.
  //
  // TODO(liang.fok) Add support for the DrakeShapes::MESH_POINTS type.
  switch (visual_element.getShape()) {
    case DrakeShapes::BOX: {
      geometry_data.type = geometry_data.BOX;
      geometry_data.num_float_data = 3;
      const auto& box = dynamic_cast<const DrakeShapes::Box&>(geometry);
      for (int i = 0; i < 3; ++i)
        geometry_data.float_data.push_back(static_cast<float>(box.size(i)));
      break;
    }
    case DrakeShapes::SPHERE: {
      geometry_data.type = geometry_data.SPHERE;
      geometry_data.num_float_data = 1;
      const auto& sphere = dynamic_cast<const DrakeShapes::Sphere&>(geometry);
      geometry_data.float_data.push_back(static_cast<float>(sphere.radius));
      break;
    }
    case DrakeShapes::CYLINDER: {
      geometry_data.type = geometry_data.CYLINDER;
      geometry_data.num_float_data = 2;
      const auto& cylinder =
          dynamic_cast<const DrakeShapes::Cylinder&>(geometry);
      geometry_data.float_data.push_back(static_cast<float>(cylinder.radius));
      geometry_data.float_data.push_back(static_cast<float>(cylinder.length));
      break;
    }
    case DrakeShapes::MESH: {
      geometry_data.type = geometry_data.MESH;
      geometry_data.num_float_data = 3;
      const auto& mesh = dynamic_cast<const DrakeShapes::Mesh&>(geometry);
      geometry_data.float_data.push_back(static_cast<float>(mesh.scale_[0]));
      geometry_data.float_data.push_back(static_cast<float>(mesh.scale_[1]));
      geometry_data.float_data.push_back(static_cast<float>(mesh.scale_[2]));

      if (mesh.uri_.find("package://") == 0) {
        geometry_data.string_data = mesh.uri_;
      } else {
        geometry_data.string_data = mesh.resolved_filename_;
      }

      break;
    }
    case DrakeShapes::CAPSULE: {
      geometry_data.type = geometry_data.CAPSULE;
      geometry_data.num_float_data = 2;
      const auto& c = dynamic_cast<const DrakeShapes::Capsule&>(geometry);
      geometry_data.float_data.push_back(static_cast<float>(c.radius));
      geometry_data.float_data.push_back(static_cast<float>(c.length));
      break;
    }
    default: {
      // Intentionally do nothing.
      break;
    }
  }

  // Saves the location and orientation of the visualization geometry in the
  // `lcmt_viewer_geometry_data` object. The location and orientation are
  // specified in the body's frame.
  Eigen::Isometry3d transform = visual_element.getLocalTransform();
  Eigen::Map<Eigen::Vector3f> position(geometry_data.position);
  position = transform.translation().cast<float>();
  Eigen::Map<Eigen::Vector4f> quaternion(geometry_data.quaternion);
  quaternion = math::RotationMatrix<double>::ToQuaternionAsVector4(
                                              transform.linear()).cast<float>();

  Eigen::Map<Eigen::Vector4f> color(geometry_data.color);
  color = visual_element.getMaterial().template cast<float>();

  return geometry_data;
}

}  // namespace rendering
}  // namespace systems
}  // namespace drake
