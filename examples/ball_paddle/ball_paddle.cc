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

namespace {

void ConstructBallPaddlePlant(
    const drake::math::RigidTransform<double>& paddle_fixed_pose,
    drake::multibody::MultibodyPlant<double>* plant,
    drake::multibody::BodyIndex* paddle_body_id,
    drake::multibody::BodyIndex* ball_body_id,
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

  plant->WeldFrames(plant->world_frame(), paddle_body.body_frame(),
                    paddle_fixed_pose);

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

}  // namespace

template <typename T>
BallPaddle<T>::BallPaddle(
    double time_step,
    const drake::math::RigidTransformd& p_WPaddle_fixed)
    : owned_plant_(std::make_unique<MultibodyPlant<double>>(time_step)),
      owned_scene_graph_(std::make_unique<SceneGraph<double>>()) {
  plant_ = owned_plant_.get();
  scene_graph_ = owned_scene_graph_.get();
  plant_->RegisterAsSourceForSceneGraph(scene_graph_);
  scene_graph_->set_name("scene_graph");
  plant_->set_name("plant");
  SetupPlant(p_WPaddle_fixed);
}

template <typename T>
void BallPaddle<T>::SetupPlant(
    const drake::math::RigidTransformd& p_WPaddle_fixed) {
  ConstructBallPaddlePlant(p_WPaddle_fixed, plant_,
                           &paddle_body_id_, &ball_body_id_,
                           &ball_sphere_geometry_id_, &paddle_box_geometry_id_);
}

template <typename T>
void BallPaddle<T>::AddToBuilder(drake::systems::DiagramBuilder<double>*
    builder) {
  builder->AddSystem(std::move(owned_plant_))->set_name("multibody_plant");
  builder->AddSystem(std::move(owned_scene_graph_))->set_name("scene_graph");
  // Connect MBP and SG.
  builder->Connect(
      plant_->get_geometry_poses_output_port(),
      scene_graph_->get_source_pose_port(plant_->get_source_id().value()));
  builder->Connect(scene_graph_->get_query_output_port(),
                   plant_->get_geometry_query_input_port());

  drake::geometry::DrakeVisualizerParams params;
  if (plant_->time_step() != 0.0) {
    // Publish every time step for discrete systems. The simulation is very
    // short and fast. We publish every time step, so we can slow down the
    // replay enough for human eyes. In a real application, you might want to
    // use the default value instead.
    params.publish_period = plant_->time_step();
  }
  params.role = drake::geometry::Role::kIllustration;
  drake::geometry::DrakeVisualizer<double>::AddToBuilder(builder, *scene_graph_,
                                                         nullptr, params);

  auto contact_results_to_lcm =
      builder
          ->template AddSystem<drake::multibody::ContactResultsToLcmSystem>(
              *plant_);
  // The simulation is very short and fast. We use 0 to publish every time
  // step, so we can slow down the replay enough for human eyes. In a real
  // application, you might want to use the default value instead.
  const double contact_publish_period = 0.0;
  auto contact_results_publisher =
      builder->AddSystem(drake::systems::lcm::LcmPublisherSystem::Make<
                         drake::lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", nullptr, contact_publish_period));
  // Contact results to lcm msg.
  builder->Connect(plant_->get_contact_results_output_port(),
                   contact_results_to_lcm->get_contact_result_input_port());
  builder->Connect(contact_results_to_lcm->get_lcm_message_output_port(),
                   contact_results_publisher->get_input_port());
}

}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::examples::BallPaddle)


