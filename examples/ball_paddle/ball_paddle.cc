#include "drake/examples/ball_paddle/ball_paddle.h"

#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/drake_visualizer_params.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
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

template <>
void ConstructBallPaddlePlant(
    double paddle_mass, double ball_mass, double ball_radius,
    const Eigen::Vector3d& paddle_size,
    const std::optional<drake::math::RigidTransform<double>>& paddle_fixed_pose,
    drake::multibody::MultibodyPlant<AutoDiffXd>* plant,
    drake::multibody::BodyIndex* paddle_body_id,
    drake::multibody::BodyIndex* ball_body_id,
    drake::multibody::JointIndex* paddle_translate_y_joint_index,
    drake::multibody::JointIndex* paddle_translate_z_joint_index,
    drake::geometry::GeometryId* ball_sphere_geometry_id,
    drake::geometry::GeometryId* paddle_box_geometry_id) {
  throw std::runtime_error("ConstructBallPaddlePlant doesn't support "
                           "AutoDiffXd yet");
}

template <>
void ConstructBallPaddlePlant(
    double paddle_mass, double ball_mass, double ball_radius,
    const Eigen::Vector3d& paddle_size,
    const std::optional<drake::math::RigidTransform<double>>& paddle_fixed_pose,
    drake::multibody::MultibodyPlant<double>* plant,
    drake::multibody::BodyIndex* paddle_body_id,
    drake::multibody::BodyIndex* ball_body_id,
    drake::multibody::JointIndex* paddle_translate_y_joint_index,
    drake::multibody::JointIndex* paddle_translate_z_joint_index,
    drake::geometry::GeometryId* ball_sphere_geometry_id,
    drake::geometry::GeometryId* paddle_box_geometry_id) {

  drake::multibody::Parser parser(plant);
  const std::string paddle_sdf_file_name =
      FindResourceOrThrow("drake/examples/ball_paddle/paddle.sdf");
  const multibody::ModelInstanceIndex model_instance_id =
      parser.AddModelFromFile(paddle_sdf_file_name);
  const multibody::RigidBody<double>& paddle_body =
      plant->GetRigidBodyByName("paddle", model_instance_id);
  *paddle_body_id = paddle_body.index();
  *paddle_box_geometry_id =
      plant->GetCollisionGeometriesForBody(paddle_body).at(0);

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

  // Now add the ball.
  const std::string ball_sdf_file_name =
      FindResourceOrThrow("drake/examples/ball_paddle/ball.sdf");
  const multibody::ModelInstanceIndex ball_instance_id =
      parser.AddModelFromFile(ball_sdf_file_name);
  const multibody::RigidBody<double>& ball_body =
      plant->GetRigidBodyByName("ball", ball_instance_id);
  *ball_body_id = ball_body.index();
  *ball_sphere_geometry_id =
      plant->GetCollisionGeometriesForBody(ball_body).at(0);

  plant->set_contact_model(
        drake::multibody::ContactModel::kHydroelasticsOnly);
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
    auto contact_results_to_lcm =
        builder
            ->template AddSystem<drake::multibody::ContactResultsToLcmSystem>(
                *plant_);
    auto contact_results_publisher =
        builder->AddSystem(drake::systems::lcm::LcmPublisherSystem::Make<
                           drake::lcmt_contact_results_for_viz>(
            "CONTACT_RESULTS", nullptr));
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

