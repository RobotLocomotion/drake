# pragma once

//#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/// A helper class to construct and run KUKA IIWA World simulations; i.e.
/// Simulations setting up the KUKA IIWA robot arm and various objects for
/// manipulation.
/// @tparam T must be a valid Eigen ScalarType.
///
/// Instantiated templates for the following ScalarTypes are provided:
/// - double
///

template <typename T>
class IiwaWorldSimulator {
  public :
  /// A constructor configuring this object to use DrakeLcm, which encapsulates
  /// a _real_ LCM instance
  IiwaWorldSimulator();
//  explicit IiwaWorldSimulator(std::unique_ptr<lcm::DrakeLcmInterface> lcm);

  ~IiwaWorldSimulator();


  /// Returns the LCM object used by this AutomotiveSimulator.
  lcm::DrakeLcmInterface* get_lcm();

  /// Returns the DiagramBuilder.
  /// @pre Start() has NOT been called.
  systems::DiagramBuilder<T>* get_builder();

  /// Returns the RigidBodyTree.  Beware that the AutomotiveSimulator::Start()
  /// method invokes RigidBodyTree::compile, which may substantially update the
  /// tree representation.
  const RigidBodyTree& get_rigid_body_tree();

  /// Adds a Iiwa arm system to this simulation.
  /// @pre Start() has NOT been called.
  int AddIiwaArm();

//  void AddObject(std::string object_name);
//
//  /// Adds an LCM publisher for the given @p system.
//  /// @pre Start() has NOT been called.
//  void AddPublisher(const SimpleCarToEulerFloatingJoint<T>& system,
//                    int vehicle_number);
//

  int AddObject();

  int allocate_object_number();

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
  void Build();

  /// Advance simulated time by the given @p time_step increment in seconds.
  void StepBy(const T& time_step);

  // We are neither copyable nor moveable.
  IiwaWorldSimulator(const IiwaWorldSimulator<T>& other) = delete;
  IiwaWorldSimulator& operator=(const IiwaWorldSimulator<T>& other) = delete;

 private:
  // For both building and simulation.
  std::unique_ptr<RigidBodyTree> rigid_body_tree_{
      std::make_unique<RigidBodyTree>()};
  lcm::DrakeLcm lcm_;

  // For building.
  std::unique_ptr<systems::DiagramBuilder<T>> builder_{
      std::make_unique<systems::DiagramBuilder<T>>()};
//  std::vector<std::pair<const RigidBody*, const systems::System<T>*>>
//      drake_visualizer_inputs_;
  int next_object_number_{0};
  bool started_{false};

  // For simulation.
  std::unique_ptr<systems::Diagram<T>> diagram_;
  std::unique_ptr<systems::Simulator<T>> simulator_;
};

}
}
}