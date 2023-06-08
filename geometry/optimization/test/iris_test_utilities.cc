#include "drake/geometry/optimization/test/iris_test_utilities.h"

#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/tree/revolute_joint.h"

namespace drake {
namespace geometry {
namespace optimization {
Toy2DofRobotTest::Toy2DofRobotTest() : builder_{} {
  builder_.plant().set_name("toy_2dof");

  ProximityProperties proximity_properties{};
  // IRIS doesn't care about robot dynamics. Use arbitrary material
  // properties.
  AddContactMaterial(0.1, 250.0, multibody::CoulombFriction<double>{0.9, 0.5},
                     &proximity_properties);
  world_left_wall_ = builder_.plant().RegisterCollisionGeometry(
      builder_.plant().world_body(),
      math::RigidTransformd(Eigen::Vector3d(-0.5, 0.5, 0)),
      geometry::Box(0.05, 1, 0.05), "left_wall", proximity_properties);
  world_right_wall_ = builder_.plant().RegisterCollisionGeometry(
      builder_.plant().world_body(),
      math::RigidTransformd(Eigen::Vector3d(0.5, 0.5, 0)),
      geometry::Box(0.05, 1, 0.05), "right_wall", proximity_properties);
  world_bottom_wall_ = builder_.plant().RegisterCollisionGeometry(
      builder_.plant().world_body(),
      math::RigidTransformd(Eigen::Vector3d(0, 0, 0)),
      geometry::Box(1, 0.05, 0.05), "bottom_wall", proximity_properties);
  world_top_wall_ = builder_.plant().RegisterCollisionGeometry(
      builder_.plant().world_body(),
      math::RigidTransformd(Eigen::Vector3d(0, 1, 0)),
      geometry::Box(1, 0.05, 0.05), "top_wall", proximity_properties);

  // IRIS only considers robot kinematics, not dynamics, So we use an arbitrary
  // inertia.
  const multibody::SpatialInertia<double> spatial_inertia(
      1, Eigen::Vector3d::Zero(),
      multibody::UnitInertia<double>(0.01, 0.01, 0.01));

  const auto& base = builder_.plant().AddRigidBody("base", spatial_inertia);
  builder_.plant().AddJoint<multibody::WeldJoint>(
      "base_weld_joint", builder_.plant().world_body(),
      math::RigidTransformd(Eigen::Vector3d(0, 0.1, 0)), base,
      math::RigidTransformd(), math::RigidTransformd());

  // body0
  body_indices_.push_back(
      builder_.plant().AddRigidBody("body0", spatial_inertia).index());
  const auto& body0 = builder_.plant().get_body(body_indices_[0]);
  const auto& joint0 = builder_.plant().AddJoint<multibody::RevoluteJoint>(
      "joint0", base, math::RigidTransformd(), body0, math::RigidTransformd(),
      Eigen::Vector3d::UnitZ());
  builder_.plant()
      .get_mutable_joint(joint0.index())
      .set_position_limits(Vector1d(-0.8 * M_PI), Vector1d(0.8 * M_PI));

  const Eigen::VectorXd y_offset0 = Eigen::VectorXd::LinSpaced(11, 0.05, 0.4);
  for (int i = 0; i < y_offset0.rows(); ++i) {
    body0_spheres_.push_back(builder_.plant().RegisterCollisionGeometry(
        body0, math::RigidTransform(Eigen::Vector3d(0, y_offset0(i), 0)),
        Sphere(0.05), fmt::format("body0_sphere{}", i), proximity_properties));
  }

  // body 1
  body_indices_.push_back(
      builder_.plant().AddRigidBody("body1", spatial_inertia).index());
  const auto& body1 = builder_.plant().get_body(body_indices_[1]);
  const auto& joint1 = builder_.plant().AddJoint<multibody::RevoluteJoint>(
      "joint1", body0, math::RigidTransformd(Eigen::Vector3d(0, 0.5, 0)), body1,
      math::RigidTransformd(), Eigen::Vector3d::UnitZ());
  builder_.plant()
      .get_mutable_joint(joint1.index())
      .set_position_limits(Vector1d(-0.8 * M_PI), Vector1d(0.8 * M_PI));

  const Eigen::VectorXd y_offset1 = Eigen::VectorXd::LinSpaced(11, 0.05, 0.45);
  for (int i = 0; i < y_offset1.rows(); ++i) {
    body1_spheres_.push_back(builder_.plant().RegisterCollisionGeometry(
        body1, math::RigidTransform(Eigen::Vector3d(0, y_offset1(i), 0)),
        Sphere(0.05), fmt::format("body1_sphere{}", i), proximity_properties));
  }

  builder_.plant().Finalize();
}
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
