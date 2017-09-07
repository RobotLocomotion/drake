#include "drake/examples/double_pendulum/test/rigid_body_types_compare.h"

#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace double_pendulum {
namespace test {

::testing::AssertionResult AreElementsEquivalent(
     const DrakeShapes::Element& elem1,
     const DrakeShapes::Element& elem2) {
  const Isometry3<double>& iso1 = elem1.getLocalTransform();
  const Isometry3<double>& iso2 = elem2.getLocalTransform();
  if (!iso1.isApprox(iso2)) {
    Eigen::IOFormat fmt(Eigen::StreamPrecision,
                        Eigen::DontAlignCols,
                        ", ", ", ", "", "", "[", "]");
    return ::testing::AssertionFailure()
        << "Element local transforms differ:"
        << " elem1 translation = " << iso1.translation().format(fmt)
        << " rotation = " << iso1.linear().format(fmt) << ","
        << " elem2 translation = " << iso2.translation().format(fmt)
        << " rotation = " << iso2.linear().format(fmt) << ".";
  }
  const DrakeShapes::Geometry& geom1 = elem1.getGeometry();
  const DrakeShapes::Geometry& geom2 = elem1.getGeometry();
  if (geom1.getShape() != geom2.getShape()) {
    return ::testing::AssertionFailure()
        << "Element geometries differ: elem1 is a "
        << DrakeShapes::ShapeToString(geom1.getShape())
        << ", elem2 is a " << DrakeShapes::ShapeToString(geom1.getShape())
        << "." << std::endl;
  }
  switch (geom1.getShape()) {
    case DrakeShapes::BOX: {
      const auto& box1 = static_cast<const DrakeShapes::Box&>(geom1);
      const auto& box2 = static_cast<const DrakeShapes::Box&>(geom2);
      if (box1.size != box2.size) {
        return ::testing::AssertionFailure()
            << "Element geometries differ: elem1 is a " << box1.size
            << " " << DrakeShapes::ShapeToString(geom1.getShape())
            << ", elem2 is a " << box2.size << " "
            << DrakeShapes::ShapeToString(geom2.getShape())
            << "." << std::endl;
      }
      break;
    }
    case DrakeShapes::CYLINDER: {
      const auto& cylinder1 = static_cast<const DrakeShapes::Cylinder&>(geom1);
      const auto& cylinder2 = static_cast<const DrakeShapes::Cylinder&>(geom2);
      if (cylinder1.radius != cylinder2.radius) {
        return ::testing::AssertionFailure()
            << "Element geometries differ: elem1 is a "
            << DrakeShapes::ShapeToString(geom1.getShape())
            << " with a " << cylinder1.radius << " radius, elem2 is a "
            << DrakeShapes::ShapeToString(geom2.getShape())
            << " with a " << cylinder2.radius << " radius." << std::endl;
      }
      if (cylinder1.length != cylinder2.length) {
        return ::testing::AssertionFailure()
            << "Element geometries differ: elem1 is a " << cylinder1.length
            << " long " << DrakeShapes::ShapeToString(geom1.getShape())
            << ", elem2 is a " << cylinder2.length << " long "
            << DrakeShapes::ShapeToString(geom2.getShape())
            << "." << std::endl;
      }
      break;
    }
    case DrakeShapes::MESH:
    case DrakeShapes::MESH_POINTS:
    case DrakeShapes::CAPSULE:
    case DrakeShapes::SPHERE:
    case DrakeShapes::UNKNOWN:
      // TODO(hidmic): Implement checks for all the other shapes.
      break;
  }
  return ::testing::AssertionSuccess()
      << "Elements elem1 and elem2 are equivalent.";
}

::testing::AssertionResult AreBodiesEquivalent(const RigidBody<double>& body1,
                                               const RigidBody<double>& body2) {
  if (body1.get_mass() != body2.get_mass()) {
    return ::testing::AssertionFailure()
        << "Body masses differ: body1 mass = " << body1.get_mass()
        << ", body2 mass = " << body2.get_mass() << ".";
  }
  if (!body1.get_center_of_mass().isApprox(body2.get_center_of_mass())) {
    Eigen::IOFormat fmt(Eigen::StreamPrecision,
                        Eigen::DontAlignCols,
                        ", ", ", ", "", "", "[", "]");
    return ::testing::AssertionFailure()
        << "Body COM differ:"
        << " body1 COM = " << body1.get_center_of_mass().format(fmt) << ","
        << " body2 COM = " << body2.get_center_of_mass().format(fmt) << ".";
  }
  const DrakeShapes::VectorOfVisualElements& body1_visuals =
      body1.get_visual_elements();
  const DrakeShapes::VectorOfVisualElements& body2_visuals =
      body2.get_visual_elements();
  if (body1_visuals.size() != body2_visuals.size()) {
    return ::testing::AssertionFailure()
        << "Body visual elements differ: body1 has "
        << body1_visuals.size()
        << " visual elements, body2 has "
        << body2_visuals.size()
        << " visual elements.";
  }
  // TODO(hidmic): do not require bodies to be built in the exact same order.
  for (int i = 0 ; i < static_cast<int>(body1_visuals.size()) ; ++i) {
    ::testing::AssertionResult result =
        AreElementsEquivalent(body1_visuals[i], body2_visuals[i]);
    if (!result) {
      return ::testing::AssertionFailure()
          << "Body visual elements differ. "
          << result.failure_message();
    }
  }
  if (body1.get_num_collision_elements() !=
      body2.get_num_collision_elements()) {
    return ::testing::AssertionFailure()
        << "Body collision elements differ: body1 has "
        << body1.get_num_collision_elements()
        << " collision elements, body2 has "
        << body2.get_num_collision_elements()
        << " collision elements.";
  }

  // Apply const_cast to call RigidBody::collision_elements_begin() and
  // RigidBody::collision_elements_end() on an initially const ref.
  RigidBody<double>& non_const_body1 = const_cast<RigidBody<double>&>(body1);
  RigidBody<double>& non_const_body2 = const_cast<RigidBody<double>&>(body2);
  // TODO(hidmic): Do not require bodies to be built in the exact same order.
  for (auto it1 = non_const_body1.collision_elements_begin(),
            it2 = non_const_body2.collision_elements_begin(),
            ed1 = non_const_body1.collision_elements_end();
       it1 != ed1 ; ++it1, ++it2) {
    const drake::multibody::collision::Element* body1_collision = *it1;
    const drake::multibody::collision::Element* body2_collision = *it2;
    ::testing::AssertionResult result =
        AreElementsEquivalent(*body1_collision, *body2_collision);
    if (!result) {
      return ::testing::AssertionFailure()
          << "Body collision elements differ. "
          << result.failure_message();
    }
  }

  return ::testing::AssertionSuccess()
      << "Bodies body1 and body2 are equivalent.";
}

}  // namespace test
}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake
