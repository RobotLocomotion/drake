#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/automotive/curve2.h"
#include "drake/automotive/dev/endless_road_car.h"
#include "drake/automotive/gen/endless_road_car_state.h"
#include "drake/automotive/endless_road_car_to_euler_floating_joint.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/utility/infinite_circuit_road.h"
#include "drake/automotive/simple_car.h"
#include "drake/automotive/simple_car_to_euler_floating_joint.h"
#include "drake/automotive/trajectory_car.h"
#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace automotive {

/// A helper class to construct and run automotive-related simulations.
///
/// @tparam T must be a valid Eigen ScalarType.
///
/// Instantiated templates for the following ScalarTypes are provided:
/// - double
///
/// They are already available to link against in the containing library.
template <typename T>
class AutomotiveSimulator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AutomotiveSimulator)

  /// A constructor that configures this object to use DrakeLcm, which
  /// encapsulates a _real_ LCM instance.
  AutomotiveSimulator();
  explicit AutomotiveSimulator(std::unique_ptr<lcm::DrakeLcmInterface> lcm);
  ~AutomotiveSimulator();

  /// Returns the LCM object used by this AutomotiveSimulator.
  lcm::DrakeLcmInterface* get_lcm();

  /// Returns the DiagramBuilder.
  /// @pre Start() has NOT been called.
  systems::DiagramBuilder<T>* get_builder();

  /// Returns the RigidBodyTree.  Beware that the AutomotiveSimulator::Start()
  /// method invokes RigidBodyTree::compile, which may substantially update the
  /// tree representation.
  const RigidBodyTree<T>& get_rigid_body_tree();

  /// Adds a SimpleCar system to this simulation, including its DrivingCommand
  /// LCM input and EulerFloatingJoint output.
  ///
  /// @pre Start() has NOT been called.
  ///
  /// @param[in] sdf_filename The name of the SDF file to load as the
  /// visualization for the simple car. This file must contain one free-floating
  /// model of a vehicle (i.e., a model that's not connected to the world). A
  /// floating joint of type multibody::joints::kRollPitchYaw is added to
  /// connect the vehicle model to the world.
  /// @param model_name  If this is non-empty, the car's model will be labeled
  ///                    with this name.
  /// @param channel_name  The simple car will subscribe to a channel
  ///                      with this name to receive commands.  Must be
  ///                      non-empty.
  ///
  /// @return The model instance ID of the SimpleCar that was just added to
  /// the simulation.
  int AddSimpleCarFromSdf(const std::string& sdf_filename,
                          const std::string& model_name,
                          const std::string& channel_name);

  /// Adds a TrajectoryCar system to this simulation, including its
  /// EulerFloatingJoint output.
  ///
  /// @pre Start() has NOT been called.
  ///
  /// @param[in] sdf_filename See the documentation for the parameter of the
  /// same name in AddSimpleCarFromSdf().
  ///
  /// @param[in] curve See documentation of TrajectoryCar::TrajectoryCar.
  ///
  /// @param[in] speed See documentation of TrajectoryCar::TrajectoryCar.
  ///
  /// @param[in] start_time See documentation of TrajectoryCar::TrajectoryCar.
  ///
  /// @return The model instance ID of the TrajectoryCar that was just added to
  /// the simulation.
  int AddTrajectoryCarFromSdf(const std::string& sdf_filename,
                              const Curve2<double>& curve,
                              double speed,
                              double start_time);

  /// Adds an EndlessRoadCar system to this simulation, including its
  /// EulerFloatingJoint output.
  ///
  /// @param id  ID string for the car instance
  /// @param sdf_filename The name of the SDF file to load as the
  ///    visualization for the simple car. This file must contain one
  ///    free-floating model of a vehicle (i.e., a model that's not connected
  ///    to the world). A floating joint of type
  ///    multibody::joints::kRollPitchYaw is added to connect the vehicle
  ///    model to the world.
  /// @param initial_state  Initial state of the car at start of simulation.
  /// @param control_type  The controller type; see EndlessRoadCar.
  /// @param channel_name  If @p control_type is kUser, then this must be
  ///                      non-empty and the car will subscribe to a channel
  ///                      with this name to receive commands.
  ///
  /// @pre Start() has NOT been called.
  /// @pre SetRoadGeometry() HAS been called.
  int AddEndlessRoadCar(
      const std::string& id,
      const std::string& sdf_filename,
      const EndlessRoadCarState<T>& initial_state,
      typename EndlessRoadCar<T>::ControlType control_type,
      const std::string& channel_name);

  /// Sets the RoadGeometry for this simulation.
  ///
  /// The provided RoadGeometry will be wrapped with in an InfiniteCircuitRoad.
  /// @p start specifies at which end of which lane the cicuit shall begin.
  /// @p path specifies the route of the circuit; if @p path is empty, some
  /// default will be constructed.  See maliput::utility::InfiniteCircuitRoad
  /// for details.
  ///
  /// @p start and @p path provide pointers to objects owned by @p road, so
  /// their lifetime requirements are dictated by @p road.
  ///
  /// @pre Start() has NOT been called.
  const maliput::utility::InfiniteCircuitRoad* SetRoadGeometry(
      std::unique_ptr<const maliput::api::RoadGeometry> road,
      const maliput::api::LaneEnd& start,
      const std::vector<const maliput::api::Lane*>& path);

  /// Adds an LCM publisher for the given @p system.
  /// @pre Start() has NOT been called.
  void AddPublisher(const EndlessRoadCar<T>& system, int vehicle_number);

  /// Adds an LCM publisher for the given @p system.
  /// @pre Start() has NOT been called.
  void AddPublisher(const SimpleCar<T>& system, int vehicle_number);

  /// Adds an LCM publisher for the given @p system.
  /// @pre Start() has NOT been called.
  void AddPublisher(const TrajectoryCar<T>& system, int vehicle_number);

  /// Adds an LCM publisher for the given @p system.
  /// @pre Start() has NOT been called.
  void AddPublisher(const EndlessRoadCarToEulerFloatingJoint<T>& system,
                    int vehicle_number);

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
  // @param target_realtime_rate This value is passed to the
  // set_target_realtime_rate method of the simulator.
  void Start(double target_realtime_rate = 0.0);

  /// Advance simulated time by the given @p time_step increment in seconds.
  void StepBy(const T& time_step);

 private:
  int allocate_vehicle_number();
  int AddSdfModel(const std::string& sdf_filename,
                  const SimpleCarToEulerFloatingJoint<T>* coord_transform,
                  const std::string& model_name);
  int AddSdfModel(const std::string& sdf_filename,
                  const EndlessRoadCarToEulerFloatingJoint<T>* coord_transform,
                  const std::string& model_name);

  // Connects the systems that output the pose of each vehicle to the
  // visualizer. This is done by using multiplexers to connect systems that
  // output constant vectors containing zero values to specify the states
  // that are not part of the vehicle poses, and the velocity states of all
  // vehicles. (The visualizer does not use the velocity state so specifying a
  // value of zero is harmless.)
  void ConnectJointStateSourcesToVisualizer();

  // Returns a vector containing the number of joint position and velocity
  // states of each model instance in rigid_body_tree_. A sequence of joint
  // position states comes first followed by a sequence of joint velocity
  // states. The length of the returned vector is thus double the number of
  // model instances since each model instance has two entries: (1) its number
  // of position states and (2) its number of velocity states.
  std::vector<int> GetModelJointStateSizes() const;

  // For both building and simulation.
  std::unique_ptr<RigidBodyTree<T>> rigid_body_tree_{
      std::make_unique<RigidBodyTree<T>>()};

  std::unique_ptr<lcm::DrakeLcmInterface> lcm_{};
  std::unique_ptr<const maliput::api::RoadGeometry> road_{};
  std::unique_ptr<const maliput::utility::InfiniteCircuitRoad> endless_road_{};
  std::map<EndlessRoadCar<T>*, EndlessRoadCarState<T>> endless_road_cars_;

  // === Start for building. ===
  std::unique_ptr<systems::DiagramBuilder<T>> builder_{
      std::make_unique<systems::DiagramBuilder<T>>()};

  // Holds information about the vehicle models being simulated. The integer is
  // the vehicle's model instance ID within the RigidBodyTree while the pointer
  // points to the system that emits the vehicle's RPY pose in the world.
  // TODO(liang.fok) Update this to support models that connect to the world
  // via non-RPY floating joints. See #3919.
  std::vector<std::pair<int, const systems::System<T>*>>
      rigid_body_tree_publisher_inputs_;
  // === End for building. ===

  int next_vehicle_number_{0};
  bool started_{false};

  // For simulation.
  std::unique_ptr<systems::Diagram<T>> diagram_;
  std::unique_ptr<systems::Simulator<T>> simulator_;
};

}  // namespace automotive
}  // namespace drake
