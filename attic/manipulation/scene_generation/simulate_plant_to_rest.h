#pragma once

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace scene_generation {

/**
 * Given a RigidBodyPlant, this class allows the construction and execution of a
 * simulation which enables the state of the plant to come to a rest from a
 * specified initial condition.
 *
 * @note The actual time taken to come to rest is very strongly dependent on
 * the kind of bodies and their inertial properties. The parameters for
 * the simulation are currently hand-tuned to bring to rest 1-30 bodies
 * of dimension and mass comparable to the manipulation targets in
 *'/examples/kuka_iiwa_arm/objects/'
 * In case the simulation were to explode, the forward simulation is simply
 * repeated with half the max step size.
 */
class SimulatePlantToRest {
 public:
  /**
   * Constructs the SimulatePlantToRest
   * @param scene_plant A unique_ptr to a RigidBodyPlant of the scene.
   * @param visualize A unique_ptr to a LeafSystem<double> that can be
   * specified to enable rendering the simulation, for eg. a DrakeVisualizer
   * system.
   */
  SimulatePlantToRest(
      std::unique_ptr<systems::RigidBodyPlant<double>> scene_plant,
      std::unique_ptr<systems::LeafSystem<double>> visualizer = {});

  /**
   * Computes simulation runs of the system starting from the configuration
   * @p q_initial and returns the final stable configuration. Internally keeps
   * repeating the run by halving the max_time_step as if the terminal velocity
   * is greater than `v_final`.
   * @param v_final A pointer to hold the resulting final velocity upon
   * completion of this simulation run.
   * @param v_threshold threshold on the velocity to terminate the execution
   * and return the terminal state.
   * @param max_settling_time is the max time to wait for settling the
   * clutter scene. Upon reaching @param max_settling_time, the simulation
   * terminates regardless of the specified @param v_threshold.
   * @returns The generalized coordinates q representing a settled configuration
   * of the RigidBodyPlant.
   */
  VectorX<double> Run(const VectorX<double>& q_initial,
                      VectorX<double>* v_final = nullptr,
                      double v_threshold = 0.1, double max_settling_time = 1.5);

  /**
   * Returns a pointer to the Sim diagram which can then be used in a variety
   * of custom simulations.
   */
  systems::Diagram<double>* GetSimDiagram();

 private:
  // Builds a diagram of the clutter scene.
  std::unique_ptr<systems::Diagram<double>> GenerateDiagram(
      std::unique_ptr<systems::RigidBodyPlant<double>> scene_plant,
      std::unique_ptr<systems::LeafSystem<double>> visualizer = {});

  systems::RigidBodyPlant<double>* plant_ptr_{nullptr};
  std::unique_ptr<systems::Diagram<double>> diagram_;
};

}  // namespace scene_generation
}  // namespace manipulation
}  // namespace drake
