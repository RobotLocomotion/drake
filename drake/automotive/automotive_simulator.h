#pragma once

#include <vector>

#include <lcm/lcm-cpp.hpp>

#include "drake/automotive/curve2.h"
#include "drake/automotive/simple_car.h"
#include "drake/automotive/simple_car_to_euler_floating_joint.h"
#include "drake/automotive/trajectory_car.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_receive_thread.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace automotive {

/// A helper class to construct and run automotive-related simulations.
///
/// @tparam T must be a valid Eigen ScalarType.
///
/// Instantiated templates for the following ScalarTypes are provided:
/// - double
///
/// They are already available to link against in libdrakeAutomotive.
template <typename T>
class AutomotiveSimulator {
 public:
  AutomotiveSimulator();
  ~AutomotiveSimulator();

  /// Returns the LCM object used by this AutomotiveSimulator.
  lcm::LCM* get_lcm();

  /// Returns the DiagramBuilder.
  /// @pre Start() has NOT been called.
  systems::DiagramBuilder<T>* get_builder();

  /// Returns the RigidBodyTree.  Beware that the AutomotiveSimulator::Start()
  /// method invokes RigidBodyTree::compile, which may substantially update the
  /// tree representation.
  const RigidBodyTree& get_rigid_body_tree();

  /// Adds a SimpleCar system to this simulation, including its DrivingCommand
  /// LCM input and EulerFloatingJoint output.
  /// @pre Start() has NOT been called.
  void AddSimpleCar();

  /// Adds a TrajectoryCar system to this simulation, including its
  /// EulerFloatingJoint output.
  /// @pre Start() has NOT been called.
  void AddTrajectoryCar(
      const Curve2<double>& curve, double speed, double start_time);

  /// Adds an LCM publisher for the given @p system.
  /// @pre Start() has NOT been called.
  void AddPublisher(const SimpleCar<T>& system, int vehicle_number);

  /// Adds an LCM publisher for the given @p system.
  /// @pre Start() has NOT been called.
  void AddPublisher(const TrajectoryCar<T>& system, int vehicle_number);

  /// Adds an LCM publisher for the given @p system.
  /// @pre Start() has NOT been called.
  void AddPublisher(const SimpleCarToEulerFloatingJoint<T>& system,
                    int vehicle_number);

  /// Take ownership of the given @p system.
  /// @pre Start() has NOT been called.
  void AddSystem(std::unique_ptr<systems::System<T>> system);

  /// Returns the System whose name matches @p name.  Throws an exception if no
  /// such system has been added, or multiple such systems have been added.
  //
  /// This is the builder variant of the method.  It can only be used prior to
  /// Start().
  ///
  /// @pre Start() has NOT been called.
  systems::System<T>& GetBuilderSystemByName(std::string name);

  /// Returns the System whose name matches @p name.  Throws an exception if no
  /// such system has been added, or multiple such systems have been added.
  ///
  /// This is the diagram variant of the method, which can only be used after
  /// Start().
  ///
  /// @pre Start() has been called.
  const systems::System<T>& GetDiagramSystemByName(std::string name) const;

  /// Build the Diagram and initialize the Simulator.  No further changes to
  /// the diagram may occur after this has been called.
  /// @pre Start() has NOT been called.
  // TODO(jwnimmer-tri) Perhaps this should be Build(), that returns an
  // AutomotiveSimulator, and our class should be AutomotiveSimulatorBuilder?
  // Port a few more demo programs, then decide what looks best.
  void Start();

  /// Advance simulated time by the given @p time_step increment in seconds.
  void StepBy(const T& time_step);

  // We are neither copyable nor moveable.
  AutomotiveSimulator(const AutomotiveSimulator<T>& other) = delete;
  AutomotiveSimulator& operator=(const AutomotiveSimulator<T>& other) = delete;

 private:
  int allocate_vehicle_number();
  void AddBoxcar(const SimpleCarToEulerFloatingJoint<T>*);

  // For both building and simulation.
  std::unique_ptr<RigidBodyTree> rigid_body_tree_{
    std::make_unique<RigidBodyTree>()};
  std::unique_ptr<lcm::LCM> lcm_{std::make_unique<lcm::LCM>()};

  // For building.
  std::unique_ptr<systems::DiagramBuilder<T>> builder_{
    std::make_unique<systems::DiagramBuilder<T>>()};
  std::vector<std::pair<const RigidBody*, const systems::System<T>*>>
      rigid_body_tree_publisher_inputs_;
  int next_vehicle_number_{0};
  bool started_{false};

  // For simulation.
  std::unique_ptr<systems::Diagram<T>> diagram_;
  std::unique_ptr<systems::Simulator<T>> simulator_;
  std::unique_ptr<systems::lcm::LcmReceiveThread> lcm_receive_thread_;
};

}  // namespace automotive
}  // namespace drake
