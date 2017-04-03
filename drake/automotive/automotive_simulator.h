#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/automotive/car_vis_applicator.h"
#include "drake/automotive/curve2.h"
#include "drake/automotive/dev/endless_road_car.h"
#include "drake/automotive/dev/endless_road_car_to_euler_floating_joint.h"
#include "drake/automotive/dev/infinite_circuit_road.h"
#include "drake/automotive/gen/endless_road_car_state.h"
#include "drake/automotive/gen/maliput_railcar_state.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput_railcar.h"
#include "drake/automotive/simple_car.h"
#include "drake/automotive/simple_car_to_euler_floating_joint.h"
#include "drake/automotive/trajectory_car.h"
#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/rendering/pose_aggregator.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

namespace drake {
namespace automotive {

/// AutomotiveSimulator is a helper class for constructing and running
/// automotive-related simulations.
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

  /// Adds a SimpleCar to this simulation visualized as a Toyota Prius. This
  /// includes its DrivingCommand LCM input and EulerFloatingJoint output.
  ///
  /// @pre Start() has NOT been called.
  ///
  /// @param model_name If this is non-empty, the car's model will be labeled
  /// with this name. It must be unique among all cars.
  ///
  /// @param channel_name  The SimpleCar will subscribe to an LCM channel of
  /// this name to receive commands.  It must be non-empty.
  ///
  /// @param initial_state The SimpleCar's initial state.
  ///
  /// @return The ID of the car that was just added to the simulation.
  int AddPriusSimpleCar(const std::string& model_name,
                        const std::string& channel_name,
                        const SimpleCarState<T>& initial_state =
                            SimpleCarState<T>());

  /// Adds a TrajectoryCar to this simulation visualized as a Toyota Prius. This
  /// includes its EulerFloatingJoint output.
  ///
  /// @pre Start() has NOT been called.
  ///
  /// @param[in] curve See documentation of TrajectoryCar::TrajectoryCar.
  ///
  /// @param[in] speed See documentation of TrajectoryCar::TrajectoryCar.
  ///
  /// @param[in] start_time See documentation of TrajectoryCar::TrajectoryCar.
  ///
  /// @return The ID of the car that was just added to the simulation.
  int AddPriusTrajectoryCar(const Curve2<double>& curve,
                            double speed,
                            double start_time);

  /// Adds an EndlessRoadCar to this simulation visualized as a Toyota Prius.
  /// This includes its EulerFloatingJoint output.
  ///
  /// @param id  ID string for the car instance
  ///
  /// @param initial_state  Initial state of the car at start of simulation.
  ///
  /// @param control_type  The controller type; see EndlessRoadCar.
  ///
  /// @param channel_name  If @p control_type is kUser, this parameter must be
  /// non-empty and the car will subscribe to an LCM channel of this name to
  /// receive commands.
  ///
  /// @return The ID of the car that was just added to the simulation.
  /// @pre Start() has NOT been called.
  /// @pre SetRoadGeometry() HAS been called.
  int AddPriusEndlessRoadCar(
      const std::string& id,
      const EndlessRoadCarState<T>& initial_state,
      typename EndlessRoadCar<T>::ControlType control_type,
      const std::string& channel_name);

  /// Adds a MaliputRailcar to this simulation visualized as a Toyota Prius.
  ///
  /// @pre Start() has NOT been called.
  ///
  /// @pre SetRoadGeometry() was called. Otherwise, a std::runtime_error will be
  /// thrown.
  ///
  /// @param model_name If this is non-empty, the car's model will be labeled
  /// with this name. It must be unique among all cars.
  ///
  /// @param initial_lane_direction The MaliputRailcar's initial lane and
  /// direction on the lane. The lane in this parameter must be part of the
  /// maliput::api::RoadGeometry that is added via SetRoadGeometry(). Otherwise
  /// a std::runtime_error will be thrown.
  ///
  /// @param params The MaliputRailcar's parameters.
  ///
  /// @param initial_state The MaliputRailcar's initial state.
  ///
  /// @return The ID of the car that was just added to the simulation.
  int AddPriusMaliputRailcar(
      const std::string& model_name,
      const LaneDirection& initial_lane_direction,
      const MaliputRailcarParams<T>& params = MaliputRailcarParams<T>(),
      const MaliputRailcarState<T>& initial_state = MaliputRailcarState<T>());

  /// Sets the acceleration command of a particular MaliputRailcar.
  ///
  /// @param id The ID of the MaliputRailcar. This is the ID that was returned
  /// by the method that added the MaliputRailcar to the simulation. If no
  /// MaliputRailcar with such an ID exists, a std::runtime_error is thrown.
  ///
  /// @param acceleration The acceleration command to issue to the
  /// MaliputRailcar.
  ///
  /// @pre Start() has been called.
  void SetMaliputRailcarAccelerationCommand(int id, double acceleration);

  /// Sets the RoadGeometry for this simulation.
  ///
  /// The provided RoadGeometry will be wrapped with in an InfiniteCircuitRoad.
  /// @p start specifies at which end of which lane the circuit shall begin.
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

  /// Sets the RoadGeometry for this simulation.
  ///
  /// @pre Start() has NOT been called.
  const maliput::api::RoadGeometry* SetRoadGeometry(
      std::unique_ptr<const maliput::api::RoadGeometry> road);

  /// Returns the System whose name matches @p name.  Throws an exception if no
  /// such system has been added, or multiple such systems have been added.
  //
  /// This is the builder variant of the method.  It can only be used prior to
  /// Start() being called.
  ///
  /// @pre Start() has NOT been called.
  systems::System<T>& GetBuilderSystemByName(std::string name);

  /// Returns the System whose name matches @p name.  Throws an exception if no
  /// such system has been added, or multiple such systems have been added.
  ///
  /// This is the diagram variant of the method, which can only be used after
  /// Start() is called.
  ///
  /// @pre Start() has been called.
  const systems::System<T>& GetDiagramSystemByName(std::string name) const;

  /// Builds the Diagram and initializes the Simulator.  No further changes to
  /// the diagram may occur after this has been called.
  ///
  /// @pre Start() has NOT been called.
  ///
  /// @param target_realtime_rate This value is passed to
  /// systems::Simulator::set_target_realtime_rate().
  //
  // TODO(jwnimmer-tri) Perhaps this should be Build(), that returns an
  // AutomotiveSimulator, and our class should be AutomotiveSimulatorBuilder?
  // Port a few more demo programs, then decide what looks best.
  void Start(double target_realtime_rate = 0.0);

  /// Returns whether the automotive simulator has started.
  bool has_started() const { return diagram_ != nullptr; }

  /// Advances simulated time by the given @p time_step increment in seconds.
  void StepBy(const T& time_step);

 private:
  int allocate_vehicle_number();

  // Adds an LCM publisher for the given @p system.
  // @pre Start() has NOT been called.
  void AddPublisher(const EndlessRoadCar<T>& system, int vehicle_number);

  // Adds an LCM publisher for the given @p system.
  // @pre Start() has NOT been called.
  void AddPublisher(const MaliputRailcar<T>& system, int vehicle_number);

  // Adds an LCM publisher for the given @p system.
  // @pre Start() has NOT been called.
  void AddPublisher(const SimpleCar<T>& system, int vehicle_number);

  // Adds an LCM publisher for the given @p system.
  // @pre Start() has NOT been called.
  void AddPublisher(const TrajectoryCar<T>& system, int vehicle_number);

  // Adds an LCM publisher for the given @p system.
  // @pre Start() has NOT been called.
  void AddPublisher(const EndlessRoadCarToEulerFloatingJoint<T>& system,
                    int vehicle_number);

  // Adds an LCM publisher for the given @p system.
  // @pre Start() has NOT been called.
  void AddPublisher(const SimpleCarToEulerFloatingJoint<T>& system,
                    int vehicle_number);

  // Takes ownership of the given @p system.
  // @pre Start() has NOT been called.
  void AddSystem(std::unique_ptr<systems::System<T>> system);

  // Generates the URDF model of the road network and loads it into the
  // `RigidBodyTree`. Member variable `road_` must be set prior to calling this
  // method.
  void GenerateAndLoadRoadNetworkUrdf();

  // Creates a lcmt_load_robot message containing all visual elements in the
  // simulation and sends it to the drake-visualizer.
  void TransmitLoadMessage();

  void SendLoadRobotMessage(const lcmt_viewer_load_robot& message);

  void InitializeSimpleCars();
  void InitializeEndlessRoadcars();
  void InitializeMaliputRailcars();

  // For both building and simulation.
  std::unique_ptr<lcm::DrakeLcmInterface> lcm_{};
  std::unique_ptr<const maliput::api::RoadGeometry> road_{};
  std::unique_ptr<const maliput::utility::InfiniteCircuitRoad> endless_road_{};

  // === Start for building. ===
  std::unique_ptr<RigidBodyTree<T>> tree_{
      std::make_unique<RigidBodyTree<T>>()};

  std::unique_ptr<systems::DiagramBuilder<T>> builder_{
      std::make_unique<systems::DiagramBuilder<T>>()};

  // Holds the desired initial states of each EndlessRoadCar. It is used to
  // initialize the simulation's diagram's state and to connect the
  // EndlessRoadCars to the EndlessRoadOracle sensor.
  std::map<const EndlessRoadCar<T>*, EndlessRoadCarState<T>> endless_road_cars_;

  // Holds the desired initial states of each SimpleCar. It is used to
  // initialize the simulation's diagram's state.
  std::map<const SimpleCar<T>*, SimpleCarState<T>> simple_car_initial_states_;

  // Holds the desired initial states of each MaliputRailcar. It is used to
  // initialize the simulation's diagram's state.
  std::map<const MaliputRailcar<T>*, std::pair<MaliputRailcarParams<T>,
                                               MaliputRailcarState<T>>>
      railcar_configs_;

  // === End for building. ===

  // Adds the PoseAggregator.
  systems::rendering::PoseAggregator<T>* aggregator_{
      builder_->template AddSystem<systems::rendering::PoseAggregator<T>>()};

  // Adds a CarVisApplicator system, which takes the poses of the vehicles and
  // outputs the poses of the visual elements that make up the visualization of
  // the vehicles. For a system-level architecture diagram, see #5541.
  CarVisApplicator<T>* car_vis_applicator_{
      builder_->template AddSystem<CarVisApplicator<T>>()};

  // Adds a PoseBundleToDrawMessage system, which takes the output of
  // car_vis_applicator_ and creates an lcmt_viewer_draw message containing the
  // latest poses of the visual elements.
  systems::rendering::PoseBundleToDrawMessage* bundle_to_draw_{
      builder_->template
          AddSystem<systems::rendering::PoseBundleToDrawMessage>()};

  // Takes the output of bundle_to_draw_ and passes it to lcm_ for publishing.
  systems::lcm::LcmPublisherSystem* lcm_publisher_{};

  int next_vehicle_number_{0};

  // Maps a vehicle id to a pointer to the system that implements the vehicle.
  std::map<int, systems::System<T>*> vehicles_;

  // For simulation.
  std::unique_ptr<systems::Diagram<T>> diagram_{};
  std::unique_ptr<systems::Simulator<T>> simulator_{};
};

}  // namespace automotive
}  // namespace drake
