#pragma once
#include <memory>
#include <optional>

#include <Eigen/Core>

#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {

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
    drake::geometry::GeometryId* paddle_box_geometry_id);

template <typename T>
class BallPaddle {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BallPaddle)

  /**
   * @param time_step The time_step for simulation. Currently the hydroelastic
   * model only supports time_step to be 0. So to use hydro-elastic model, set
   * time_step=0.
   * @param p_WPaddle_fixed The fixed pose of the paddle in the world frame.
   * Default to std::nullopt, which means the paddle is not fixed.
   */
  explicit BallPaddle(double time_step,
                      const std::optional<drake::math::RigidTransformd>&
                          p_WPaddle_fixed = std::nullopt);

  const drake::multibody::MultibodyPlant<T>& plant() const { return *plant_; }

  const drake::geometry::SceneGraph<T>& scene_graph() const {
    return *scene_graph_;
  }

  drake::geometry::GeometryId ball_sphere_geometry_id() const {
    return ball_sphere_geometry_id_;
  }

  drake::geometry::GeometryId paddle_box_geometry_id() const {
    return paddle_box_geometry_id_;
  }

  const drake::multibody::Body<T>& paddle_body() const {
    return plant_->get_body(paddle_body_id_);
  }

  const drake::multibody::Body<T>& ball_body() const {
    return plant_->get_body(ball_body_id_);
  }

  const drake::multibody::Joint<T>& paddle_translate_y_joint() const {
    return plant_->get_joint(paddle_translate_y_joint_index_);
  }

  const drake::multibody::Joint<T>& paddle_translate_z_joint() const {
    return plant_->get_joint(paddle_translate_z_joint_index_);
  }

  /**
   * Builds a diagram that contains the ball paddle MultibodyPlant and
   * SceneGraph. Note that we don't call builder.Build() inside this function,
   * since this function doesn't build the controller yet.
   */
  void AddToBuilder(drake::systems::DiagramBuilder<T>* builder);

  double ball_mass() const { return ball_mass_; }

  double paddle_mass() const { return paddle_mass_; }

  double ball_radius() const { return ball_radius_; }

  const Eigen::Vector3d& paddle_size() const { return paddle_size_; }

 private:
  void SetupPlant(
      const std::optional<drake::math::RigidTransformd>& p_WPaddle_fixed);

  const double paddle_mass_{5000.};
  const double ball_mass_{0.1};
  const double ball_radius_{0.02};
  const Eigen::Vector3d paddle_size_{1, 1, 0.4};

  std::unique_ptr<drake::multibody::MultibodyPlant<T>> owned_plant_;
  std::unique_ptr<drake::geometry::SceneGraph<T>> owned_scene_graph_;

  drake::multibody::MultibodyPlant<T>* plant_;
  drake::geometry::SceneGraph<T>* scene_graph_;

  drake::multibody::BodyIndex paddle_body_id_;
  drake::multibody::BodyIndex ball_body_id_;

  drake::multibody::JointIndex paddle_translate_y_joint_index_;
  drake::multibody::JointIndex paddle_translate_z_joint_index_;

  drake::geometry::GeometryId ball_sphere_geometry_id_;
  drake::geometry::GeometryId paddle_box_geometry_id_;
};
}  // namespace examples
}  // namespace drake
