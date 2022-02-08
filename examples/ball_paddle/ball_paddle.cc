#include "drake/examples/ball_paddle/ball_paddle.h"

#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/drake_visualizer_params.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"

namespace drake {
namespace examples {

using drake::geometry::SceneGraph;
using drake::math::RigidTransformd;
using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;
using drake::multibody::PrismaticJoint;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;

template <typename T>
void ConstructBallPaddlePlant(
    double paddle_mass, double ball_mass, double ball_radius,
    const Eigen::Vector3d& paddle_size,
    const std::optional<drake::math::RigidTransform<double>>& paddle_fixed_pose,
    drake::multibody::MultibodyPlant<T>* plant,
    drake::multibody::BodyIndex* paddle_body_id,
    drake::multibody::BodyIndex* ball_body_id,
    drake::multibody::JointIndex* paddle_translate_y_joint_index,
    drake::multibody::JointIndex* paddle_translate_z_joint_index,
    drake::geometry::GeometryId* ball_sphere_geometry_id,
    drake::geometry::GeometryId* paddle_box_geometry_id) {
  const SpatialInertia<double> paddle_inertia(
      paddle_mass, Eigen::Vector3d::Zero(), UnitInertia<double>(1., 1., 1.));
  const auto& paddle_body = plant->AddRigidBody("paddle", paddle_inertia);
  *paddle_body_id = paddle_body.index();
  const drake::geometry::Box paddle_box(paddle_size(0), paddle_size(1),
                                        paddle_size(2));
  if (paddle_fixed_pose.has_value()) {
    plant->WeldFrames(plant->world_frame(), paddle_body.body_frame(),
                      paddle_fixed_pose.value());
  } else {
    // Add a massless dummy body, so that the connection from the world to
    // paddle_body is world -> prismatic_joint_y -> paddle_dummy ->
    // prismatic_joint_z -> paddle_body.
    const auto& paddle_dummy = plant->AddRigidBody(
        "paddle_dummy", SpatialInertia<double>(0., Eigen::Vector3d::Zero(),
                                               UnitInertia<double>(0, 0, 0)));
    const auto& paddle_translate_y_joint =
        plant->template AddJoint<PrismaticJoint>(
            "paddle_translate_y", plant->world_body(), std::nullopt,
            paddle_dummy, std::nullopt, Eigen::Vector3d::UnitY());
    *paddle_translate_y_joint_index = paddle_translate_y_joint.index();
    const auto& paddle_translate_z_joint =
        plant->template AddJoint<PrismaticJoint>(
            "paddle_translate_z", paddle_dummy, std::nullopt, paddle_body,
            std::nullopt, Eigen::Vector3d::UnitZ());
    *paddle_translate_z_joint_index = paddle_translate_z_joint.index();
    const double paddle_actuator_effort_limit = 20;
    plant->AddJointActuator("paddle_y_actuactor", paddle_translate_y_joint,
                            paddle_actuator_effort_limit);
    plant->AddJointActuator("paddle_z_actuactor", paddle_translate_z_joint,
                            paddle_actuator_effort_limit);
  }

  plant->RegisterVisualGeometry(paddle_body, RigidTransformd::Identity(),
                                paddle_box, "paddle",
                                Eigen::Vector4d(1, 0.64, 0, 0.5));
  const CoulombFriction<double> paddle_friction(1., 1.);
  drake::geometry::ProximityProperties paddle_properties;
  drake::geometry::AddCompliantHydroelasticProperties(100, 1e6,
                                                      &paddle_properties);
  drake::geometry::AddContactMaterial(/* dissipation */ std::nullopt,
                                      /* point_stiffness */ std::nullopt,
                                      paddle_friction, &paddle_properties);
  *paddle_box_geometry_id = plant->RegisterCollisionGeometry(
      paddle_body, RigidTransformd::Identity(), paddle_box, "paddle_box",
      std::move(paddle_properties));

  // Now add the ball.
  const SpatialInertia<double> ball_inertia(ball_mass, Eigen::Vector3d::Zero(),
                                            UnitInertia<double>(1, 1, 1));
  const auto& ball_body = plant->AddRigidBody("ball", ball_inertia);
  *ball_body_id = ball_body.index();
  const drake::geometry::Sphere ball_sphere(ball_radius);
  plant->RegisterVisualGeometry(ball_body, RigidTransformd::Identity(),
                                ball_sphere, "ball",
                                Eigen::Vector4d(0.5, 1, 0, 0.5));
  const CoulombFriction<double> ball_friction(1., 1.);
  const T ball_stiffness = T(980);
  // Little dissipation means the ball can bounce off.
  const bool hydroelastic = false;
  const T ball_dissipation = hydroelastic ? T(0.1) : T(0.1);
  drake::geometry::ProximityProperties ball_properties;
  drake::geometry::AddRigidHydroelasticProperties(100, &ball_properties);
  ball_properties.AddProperty(drake::geometry::internal::kMaterialGroup,
                              drake::geometry::internal::kFriction,
                              ball_friction);
  ball_properties.AddProperty(drake::geometry::internal::kMaterialGroup,
                              drake::geometry::internal::kPointStiffness,
                              ball_stiffness);
  ball_properties.AddProperty(drake::geometry::internal::kMaterialGroup,
                              drake::geometry::internal::kHcDissipation,
                              ball_dissipation);
  ball_properties.AddProperty(drake::geometry::internal::kMaterialGroup,
                              drake::geometry::internal::kElastic, 1E10);
  *ball_sphere_geometry_id = plant->RegisterCollisionGeometry(
      ball_body, RigidTransformd::Identity(), ball_sphere, "ball_sphere",
      std::move(ball_properties));
  if (hydroelastic) {
    plant->set_contact_model(
        drake::multibody::ContactModel::kHydroelasticsOnly);
  }
  plant->Finalize();
}

template <typename T>
BallPaddle<T>::BallPaddle(
    double time_step,
    const std::optional<drake::math::RigidTransformd>& p_WPaddle_fixed)
    : owned_plant_(std::make_unique<MultibodyPlant<T>>(time_step)),
      owned_scene_graph_(std::make_unique<SceneGraph<T>>()) {
  plant_ = owned_plant_.get();
  scene_graph_ = owned_scene_graph_.get();
  plant_->RegisterAsSourceForSceneGraph(scene_graph_);
  scene_graph_->set_name("scene_graph");
  plant_->set_name("plant");
  SetupPlant(p_WPaddle_fixed);
}

template <typename T>
void BallPaddle<T>::SetupPlant(
    const std::optional<drake::math::RigidTransformd>& p_WPaddle_fixed) {
  ConstructBallPaddlePlant(paddle_mass_, ball_mass_, ball_radius_, paddle_size_,
                           p_WPaddle_fixed, plant_, &paddle_body_id_,
                           &ball_body_id_, &paddle_translate_y_joint_index_,
                           &paddle_translate_z_joint_index_,
                           &ball_sphere_geometry_id_, &paddle_box_geometry_id_);
}

template <typename T>
void BallPaddle<T>::AddToBuilder(drake::systems::DiagramBuilder<T>* builder) {
  builder->AddSystem(std::move(owned_plant_))->set_name("multibody_plant");
  builder->AddSystem(std::move(owned_scene_graph_))->set_name("scene_graph");
  // Connect MBP and SG.
  builder->Connect(
      plant_->get_geometry_poses_output_port(),
      scene_graph_->get_source_pose_port(plant_->get_source_id().value()));
  builder->Connect(scene_graph_->get_query_output_port(),
                   plant_->get_geometry_query_input_port());

  drake::geometry::DrakeVisualizerParams params;
  params.role = drake::geometry::Role::kIllustration;
  drake::geometry::DrakeVisualizer<T>::AddToBuilder(builder, *scene_graph_,
                                                    nullptr, params);

  if constexpr (std::is_same_v<T, double>) {
    const double publish_period = 0.001;
    auto contact_results_to_lcm =
        builder
            ->template AddSystem<drake::multibody::ContactResultsToLcmSystem>(
                *plant_);
    auto contact_results_publisher =
        builder->AddSystem(drake::systems::lcm::LcmPublisherSystem::Make<
                           drake::lcmt_contact_results_for_viz>(
            "CONTACT_RESULTS", nullptr, publish_period));
    // Contact results to lcm msg.
    builder->Connect(plant_->get_contact_results_output_port(),
                     contact_results_to_lcm->get_contact_result_input_port());
    builder->Connect(contact_results_to_lcm->get_lcm_message_output_port(),
                     contact_results_publisher->get_input_port());
  }
}

}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&drake::examples::ConstructBallPaddlePlant<T>))
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::examples::BallPaddle)
