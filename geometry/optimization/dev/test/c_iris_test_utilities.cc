#include "drake/geometry/optimization/dev/test/c_iris_test_utilities.h"

#include <string>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace geometry {
namespace optimization {
CIrisToyRobotTest::CIrisToyRobotTest() {
  systems::DiagramBuilder<double> builder;
  plant_ = builder.AddSystem<multibody::MultibodyPlant<double>>(0.);
  scene_graph_ = builder.AddSystem<geometry::SceneGraph<double>>();
  plant_->RegisterAsSourceForSceneGraph(scene_graph_);

  builder.Connect(scene_graph_->get_query_output_port(),
                  plant_->get_geometry_query_input_port());
  builder.Connect(
      plant_->get_geometry_poses_output_port(),
      scene_graph_->get_source_pose_port(plant_->get_source_id().value()));

  ProximityProperties proximity_properties{};
  // C-IRIS doesn't care about robot dynamics. Use arbitrary material
  // properties.
  AddContactMaterial(0.1, 250.0, multibody::CoulombFriction<double>{0.9, 0.5},
                     &proximity_properties);

  world_box_ = plant_->RegisterCollisionGeometry(
      plant_->world_body(),
      math::RigidTransform(math::RollPitchYawd(0.5, 0.2, -0.3),
                           Eigen::Vector3d(0.2, -0.5, 0.1)),
      geometry::Box(0.05, 0.03, 0.05), "world_box", proximity_properties);
  world_sphere_ = plant_->RegisterCollisionGeometry(
      plant_->world_body(),
      math::RigidTransform(Eigen::Vector3d(-0.1, -0.5, 0.2)), Sphere(0.04),
      "world_sphere", proximity_properties);

  // C-IRIS only considers robot kinematics, not dynamics. So we use an
  // arbitrary inertia.
  const multibody::SpatialInertia<double> spatial_inertia(
      1, Eigen::Vector3d::Zero(),
      multibody::UnitInertia<double>(0.01, 0.01, 0.01, 0, 0, 0));

  // body0
  body_indices_.push_back(
      plant_->AddRigidBody("body0", spatial_inertia).index());
  const multibody::Body<double>& body0 = plant_->get_body(body_indices_[0]);
  plant_->AddJoint<multibody::WeldJoint>(
      "joint0", plant_->world_body(),
      math::RigidTransformd(Eigen::Vector3d(0.1, 0.2, 0)), body0,
      math::RigidTransformd(math::RollPitchYawd(0.1, 0.5, 0.2),
                            Eigen::Vector3d::Zero()),
      math::RigidTransformd(Eigen::Vector3d(0.05, 0.1, 0.05)));
  body0_box_ = plant_->RegisterCollisionGeometry(
      body0, math::RigidTransform(Eigen::Vector3d(0.1, 0.05, -0.05)),
      Box(0.05, 0.03, 0.04), "body0_box", proximity_properties);
  body0_sphere_ = plant_->RegisterCollisionGeometry(
      body0, math::RigidTransform(Eigen::Vector3d(0.2, -0.02, 0)), Sphere(0.04),
      "body0_sphere", proximity_properties);

  // body1
  body_indices_.push_back(
      plant_->AddRigidBody("body1", spatial_inertia).index());
  const multibody::Body<double>& body1 = plant_->get_body(body_indices_[1]);
  const auto& joint1 = plant_->AddJoint<multibody::RevoluteJoint>(
      "joint1", body0,
      math::RigidTransformd(math::RollPitchYawd(0.1, 0.2, -0.1),
                            Eigen::Vector3d(0.2, 0.4, 0.1)),
      body1, math::RigidTransformd(Eigen::Vector3d(0.1, 0.2, -0.05)),
      Eigen::Vector3d::UnitY());
  plant_->get_mutable_joint(joint1.index())
      .set_position_limits(Vector1d(-0.5 * M_PI), Vector1d(0.7 * M_PI));
  body1_capsule_ = plant_->RegisterCollisionGeometry(
      body1, math::RigidTransformd(Eigen::Vector3d(0.2, -0.1, 0.05)),
      Capsule(0.04, 0.2), "body1_capsule", proximity_properties);
  const std::string convex_obj =
      FindResourceOrThrow("drake/geometry/optimization/dev/test/convex.obj");
  body1_convex_ = plant_->RegisterCollisionGeometry(
      body1,
      math::RigidTransformd(math::RollPitchYawd(0.05, -0.03, 0),
                            Eigen::Vector3d(0.04, 0.02, 0.05)),
      Convex(convex_obj), "body1_convex", proximity_properties);

  // body2
  body_indices_.push_back(
      plant_->AddRigidBody("body2", spatial_inertia).index());
  const auto& body2 = plant_->get_body(body_indices_[2]);
  const auto& joint2 = plant_->AddJoint<multibody::PrismaticJoint>(
      "joint2", body1, math::RigidTransformd(Eigen::Vector3d(0.2, 0, 0)), body2,
      math::RigidTransformd(math::RollPitchYawd(0.1, -0.2, 0.1),
                            Eigen::Vector3d(0.2, 0.1, 0.03)),
      Eigen::Vector3d::UnitX());
  plant_->get_mutable_joint(joint2.index())
      .set_position_limits(Vector1d(-0.2), Vector1d(0.3));
  body2_capsule_ = plant_->RegisterCollisionGeometry(
      body2, math::RigidTransform(Eigen::Vector3d(0.02, 0.05, 0)),
      Capsule(0.03, 0.05), "body2_capsule", proximity_properties);
  body2_sphere_ = plant_->RegisterCollisionGeometry(
      body2, math::RigidTransform(Eigen::Vector3d(0.01, 0.04, 0.02)),
      Sphere(0.04), "body2_sphere", proximity_properties);

  // body3
  body_indices_.push_back(
      plant_->AddRigidBody("body3", spatial_inertia).index());
  const auto& body3 = plant_->get_body(body_indices_[3]);
  const auto& joint3 = plant_->AddJoint<multibody::RevoluteJoint>(
      "joint3", body0, math::RigidTransformd(Eigen::Vector3d(0, 0.05, 0.1)),
      body3,
      math::RigidTransformd(math::RollPitchYawd(0.1, -0.1, 0.2),
                            Eigen::Vector3d(0.1, 0.2, -0.05)),
      Eigen::Vector3d::UnitY());
  plant_->get_mutable_joint(joint3.index())
      .set_position_limits(Vector1d(-0.7 * M_PI), Vector1d(0.6 * M_PI));
  body3_box_ = plant_->RegisterCollisionGeometry(
      body3, math::RigidTransformd(Eigen::Vector3d(0.1, -0.1, 0.02)),
      Box(0.2, 0.05, 0.1), "body3_box", proximity_properties);
  body3_sphere_ = plant_->RegisterCollisionGeometry(
      body3, math::RigidTransformd(Eigen::Vector3d(0.1, 0.02, 0.2)),
      Sphere(0.04), "body3_sphere", proximity_properties);

  plant_->Finalize();
  diagram_ = builder.Build();
}
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
