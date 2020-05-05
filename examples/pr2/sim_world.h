#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/examples/pr2/model_instance_info.h"
#include "drake/examples/pr2/robot_parameters.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace pr2 {

/// A system that represents the complete simulation world environment,
/// including the robots and anything a user might want to load into the model
/// such as objects or sensors.
///
/// @system{SimWorld,
///   @output_port{[name]_position_measured}
///   @output_port{[name]_velocity_estimated}
///   @output_port{[name]_state_estimated}
///   @output_port{pose_bundle}
///   @output_port{contact_results}
///   @output_port{geometry_poses}
/// }
/// Note that, the exact name of the port will depend on the name of the items
/// since the world may contain more than one robot and more than one sensors.
///
/// This class is designed to have the capability to work with different kinds
/// of mobile based robot. It will be considered to move to an independent
/// folder once we have more than one mobile base robot in the examples.

template <typename T>
class SimWorld : public systems::Diagram<T> {
 public:
  /// Internal plants created for control purposes. The welded version of the
  /// same robot plant is created for the purpose of using the inverse
  /// dynamics controller (IDC), which only works for fully actuated systems
  /// since the Drake IDC only supports fully actuated systems for now. Future
  /// work could add other controllers that do not require fully actuation.
  /// Robots with a mobile base are underactuated. We have to weld the base to
  /// the ground or someplace to get a fully actuated system, i.e., each moving
  /// joint should have one and only actuator associated to it. Welding the base
  /// of a mobile robot to the ground does not necessarily yield a fully
  /// actuated system though. However, if the users want to use the IDC, this
  /// assumption has to be true. Otherwise, a vanilla PID controller or PID with
  /// gravity compensation controller can be used.
  ///
  /// The floating base robot (or the original robot plant) is necessary to map
  /// the states of the welded plant back to the states of the original plant.
  /// Then the states can be again easily inserted back to the state of the
  /// full `sim_world` system. The original floating robot plant can also be
  /// useful for many other situations such as reading and setting
  /// joint commands, etc.
  struct OwnedRobotControllerPlant {
    using mbp = multibody::MultibodyPlant<T>;
    explicit OwnedRobotControllerPlant(const double max_time_step)
        : float_plant(std::make_unique<mbp>(max_time_step)),
          welded_plant(std::make_unique<mbp>(max_time_step)) {}
    std::unique_ptr<mbp> float_plant;
    std::unique_ptr<mbp> welded_plant;
  };

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimWorld);
  /// @param robot_names A list of the names of the robots that will be loaded.
  /// The default is one robot of `pr2`. It will throw if the vector is empty.
  explicit SimWorld(const std::vector<std::string>& robot_names =
                        std::vector<std::string>{"pr2"});

  /// Sets the default State using the Diagram's default state but override
  /// free body positions with data from the model directive yaml files and the
  /// default joint positions and velocities of the robot from
  /// GetModelPositionState() and GetModelVelocityState().
  /// @param context A const reference to the SimWorld context.
  /// @param state A pointer to the State of the SimWorld system.
  /// @pre `state` must be the systems::State<T> object contained in `context`.
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override;

  /// Returns a reference to the robot plant used for the internal controller
  /// calculation purpose. If the given robot (by name) does not exist, it will
  /// throw.
  /// @param robot_name Name of the inquired robot.
  const multibody::MultibodyPlant<T>& get_robot_plant(
      const std::string& robot_name) const {
    const auto& robot_owned_plant = owned_robots_plants_.find(robot_name);
    DRAKE_DEMAND(robot_owned_plant != owned_robots_plants_.end());
    return *(robot_owned_plant->second.float_plant);
  }

  /// Returns a reference to the main plant responsible for the dynamics of
  /// the robot and the environment.  This can be used to, e.g., add
  /// additional elements into the world before calling Finalize().
  const multibody::MultibodyPlant<T>& get_sim_world_plant() const {
    return *plant_;
  }

  /// Returns a mutable reference to the main plant responsible for the
  /// dynamics of the robot and the environment.  This can be used to, e.g.,
  /// add additional elements into the world before calling Finalize().
  multibody::MultibodyPlant<T>& get_mutable_sim_world_plant() {
    return *plant_;
  }

  /// Returns a reference to the SceneGraph responsible for all of the geometry
  /// for the robot and the environment.  This can be used to, e.g., add
  /// additional elements into the world before calling Finalize().
  const geometry::SceneGraph<T>& get_scene_graph() const {
    return *scene_graph_;
  }

  /// Returns a mutable reference to the SceneGraph responsible for all of the
  /// geometry for the robot and the environment.  This can be used to, e.g.,
  /// add additional elements into the world before calling Finalize().
  geometry::SceneGraph<T>& get_mutable_scene_graph() { return *scene_graph_; }

  /// Gets the measured position state with a given model index.
  /// @param context A const reference to the SimWorld context.
  /// @param model_index Model index of the interested model.
  VectorX<T> GetModelPositionState(
      const systems::Context<T>& context,
      const multibody::ModelInstanceIndex& model_index) const;

  /// Gets the estimated velocity state with a given model index.
  /// @param context A const reference to the SimWorld context.
  /// @param model_index Model index of the interested model.
  VectorX<T> GetModelVelocityState(
      const systems::Context<T>& context,
      const multibody::ModelInstanceIndex& model_index) const;

  /// Set the position state of a model that has at least one free moving joint.
  /// For example, the model could be a robot or a dishwasher.
  /// @param context A const reference to the SimWorld context.
  /// @param model_index Const reference of the interested model instance index.
  /// @param q The target position state of the corresponding model to be set.
  /// @param state The full state of the robot world, must be the
  /// systems::State<T> object contained in `context`.
  void SetModelPositionState(const systems::Context<T>& context,
                             const multibody::ModelInstanceIndex& model_index,
                             const Eigen::Ref<const VectorX<T>>& q,
                             systems::State<T>* state) const;

  /// Set the velocity state of a model that has at least one free moving joint.
  /// For example, the model could be a robot or a dishwasher.
  /// @param context A const reference to the SimWorld context.
  /// @param model_index Const reference of the interested model instance index.
  /// @param v The target velocity state of the corresponding model to be set.
  /// @param state The full state of the SimWorld, must be the
  /// systems::State<T> object contained in `context`.
  void SetModelVelocityState(const systems::Context<T>& context,
                             const multibody::ModelInstanceIndex& model_index,
                             const Eigen::Ref<const VectorX<T>>& v,
                             systems::State<T>* state) const;

 private:
  // Finalize the multibody plant and before using this class in the Systems
  // framework. This should be called exactly once. See
  // multibody::MultibodyPlant<T>::Finalize().
  void Finalize();

  void LoadModelsFromUrdfs();

  // Load the robot parameters with a given @p`robot_name`.
  // @throw std::exception if the robot parameter file of the given robot name
  // does not exists.
  const pr2::RobotParameters LoadRobotParameters(
      const std::string& robot_name) const;

  void AddRobotModel(const std::string& robot_name);

  // Post processing the loaded models such as setup the initial position of
  // items.
  void SetupSimWorld();

  // Create two models for controller purpose. One model is the original model
  // and one model is the welded version of the original model, independent
  // of whether the original model is welded or not. If the original model is
  // already a fixed-base robot, the original version and the welded version
  // will be the same. However, here we keep two models to make sure the logic
  // works for both fixed-base models and floating-base models. This function
  // assumes fmk_model_info_ has already being populated. Should only be called
  // from Finalize().
  void MakeRobotControlPlants();

  const std::vector<std::string> robot_names_;

  // These are only valid until Finalize() is called.
  std::unique_ptr<multibody::MultibodyPlant<T>> owned_plant_;
  std::unique_ptr<geometry::SceneGraph<T>> owned_scene_graph_;

  // These are valid for the lifetime of this system.
  multibody::MultibodyPlant<T>* plant_{};
  geometry::SceneGraph<T>* scene_graph_{};

  std::map<std::string, RobotParameters> robots_parameters_;

  /// Create internal plants for the robots.
  std::map<std::string, OwnedRobotControllerPlant> owned_robots_plants_;
};

}  // namespace pr2
}  // namespace examples
}  // namespace drake
