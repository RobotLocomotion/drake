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
      geometry::Box(0.02, 0.03, 0.01), "world_box", proximity_properties);
  world_cylinder_ = plant_->RegisterCollisionGeometry(
      plant_->world_body(),
      math::RigidTransform(Eigen::Vector3d(-0.1, -0.1, 0.2)),
      Cylinder(0.02, 0.1), "world_cylinder", proximity_properties);

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
      Box(0.05, 0.1, 0.04), "body0_box", proximity_properties);
  body0_sphere_ = plant_->RegisterCollisionGeometry(
      body0, math::RigidTransform(Eigen::Vector3d(0.01, -0.02, 0)),
      Sphere(0.08), "body0_sphere", proximity_properties);

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
      .set_position_limits(Vector1d(-0.8 * M_PI), Vector1d(0.7 * M_PI));
  body1_capsule_ = plant_->RegisterCollisionGeometry(
      body1, math::RigidTransformd(Eigen::Vector3d(0.02, -0.1, 0.05)),
      Capsule(0.08, 0.2), "body1_capsule", proximity_properties);
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
                            Eigen::Vector3d(0.02, 0.1, 0.03)),
      Eigen::Vector3d::UnitX());
  plant_->get_mutable_joint(joint2.index())
      .set_position_limits(Vector1d(-2.4), Vector1d(2.9));
  body2_capsule_ = plant_->RegisterCollisionGeometry(
      body2, math::RigidTransform(Eigen::Vector3d(0.02, 0.05, 0)),
      Capsule(0.06, 0.1), "body2_capsule", proximity_properties);
  body2_sphere_ = plant_->RegisterCollisionGeometry(
      body2, math::RigidTransform(Eigen::Vector3d(0.01, 0.04, 0.02)),
      Sphere(0.07), "body2_sphere", proximity_properties);

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
      body3, math::RigidTransformd(Eigen::Vector3d(-0.1, -0.1, 0.02)),
      Box(0.02, 0.05, 0.02), "body3_box", proximity_properties);
  body3_cylinder_ = plant_->RegisterCollisionGeometry(
      body3, math::RigidTransformd(Eigen::Vector3d(0.1, 0.02, 0.2)),
      Cylinder(0.04, 0.05), "body3_cylinder", proximity_properties);

  plant_->Finalize();
  diagram_ = builder.Build();
}

CIrisRobotPolytopicGeometryTest::CIrisRobotPolytopicGeometryTest() {
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

  world_boxes_.push_back(plant_->RegisterCollisionGeometry(
      plant_->world_body(),
      math::RigidTransform(math::RollPitchYawd(0.5, 0.2, -0.3),
                           Eigen::Vector3d(0.2, -0.5, 0.1)),
      geometry::Box(0.02, 0.03, 0.01), "world_box0", proximity_properties));
  world_boxes_.push_back(plant_->RegisterCollisionGeometry(
      plant_->world_body(),
      math::RigidTransform(math::RollPitchYawd(0.1, 0.2, -0.),
                           Eigen::Vector3d(0.2, 0.3, 0.1)),
      geometry::Box(0.02, 0.1, 0.05), "world_box1", proximity_properties));
  world_boxes_.push_back(plant_->RegisterCollisionGeometry(
      plant_->world_body(),
      math::RigidTransform(math::RollPitchYawd(0.1, 0.2, -0.),
                           Eigen::Vector3d(0.2, 0.2, 0.1)),
      geometry::Box(0.04, 0.1, 0.05), "world_box2", proximity_properties));
  const std::string convex_obj =
      FindResourceOrThrow("drake/geometry/optimization/dev/test/convex.obj");
  world_convex_ = plant_->RegisterCollisionGeometry(
      plant_->world_body(),
      math::RigidTransform(Eigen::Vector3d(-0.1, -0.5, 0.2)),
      Convex(convex_obj), "world_convex", proximity_properties);

  // C-IRIS only considers robot kinematics, not dynamics. So we use an
  // arbitrary inertia.
  const multibody::SpatialInertia<double> spatial_inertia(
      1, Eigen::Vector3d::Zero(),
      multibody::UnitInertia<double>(0.01, 0.01, 0.01, 0, 0, 0));

  auto add_body = [this, &spatial_inertia, &proximity_properties](
                      const math::RigidTransformd& X_PF,
                      const math::RigidTransformd& X_BM,
                      const Eigen::Vector3d& axis, double theta_lower,
                      double theta_upper, const math::RigidTransformd& X_BG,
                      const Eigen::Vector3d& box_size) {
    const int body_index = this->body_indices_.size();
    const auto& parent_body =
        this->body_indices_.empty()
            ? this->plant_->world_body()
            : this->plant_->get_body(this->body_indices_.back());
    this->body_indices_.push_back(
        this->plant_
            ->AddRigidBody("body" + std::to_string(body_index), spatial_inertia)
            .index());

    const auto& body = this->plant_->get_body(this->body_indices_.back());
    const auto& joint = this->plant_->AddJoint<multibody::RevoluteJoint>(
        "joint" + std::to_string(body_index), parent_body, X_PF, body, X_BM,
        axis);
    plant_->get_mutable_joint(joint.index())
        .set_position_limits(Vector1d(theta_lower), Vector1d(theta_upper));
    this->body_boxes_.push_back(this->plant_->RegisterCollisionGeometry(
        body, X_BG, Box(box_size(0), box_size(1), box_size(2)),
        "body" + std::to_string(body_index) + "_box", proximity_properties));
  };

  // body0
  add_body(math::RigidTransformd(Eigen::Vector3d(0.1, 0.2, 0)),
           math::RigidTransformd(Eigen::Vector3d(0.05, 0.01, 0)),
           Eigen::Vector3d::UnitX(), -0.5 * M_PI, 0.8 * M_PI,
           math::RigidTransformd(Eigen::Vector3d(0.05, 0.1, 0)),
           Eigen::Vector3d(0.1, 0.15, 0.1));
  // body1
  add_body(math::RigidTransformd(Eigen::Vector3d(0.05, 0, 0.1)),
           math::RigidTransformd(), Eigen::Vector3d::UnitY(), -0.6 * M_PI,
           0.7 * M_PI, math::RigidTransformd(Eigen::Vector3d(0.02, 0, 0.05)),
           Eigen::Vector3d(0.04, 0.05, 0.08));
  // body2
  add_body(math::RigidTransformd(Eigen::Vector3d(0., 0.2, 0.1)),
           math::RigidTransformd(), Eigen::Vector3d::UnitX(), -0.3 * M_PI,
           0.7 * M_PI, math::RigidTransformd(Eigen::Vector3d(0.0, 0.1, 0.05)),
           Eigen::Vector3d(0.04, 0.1, 0.08));
  // body3
  add_body(math::RigidTransformd(Eigen::Vector3d(0.1, 0.2, -0.1)),
           math::RigidTransformd(), Eigen::Vector3d::UnitZ(), -0.5 * M_PI,
           0.7 * M_PI, math::RigidTransformd(Eigen::Vector3d(0.0, 0.1, 0.04)),
           Eigen::Vector3d(0.05, 0.1, 0.02));

  plant_->Finalize();
  diagram_ = builder.Build();
}
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
