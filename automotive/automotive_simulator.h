#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/automotive/curve2.h"
#include "drake/automotive/gen/maliput_railcar_state.h"
#include "drake/automotive/gen/trajectory_car_state.h"
#include "drake/automotive/idm_controller.h"
#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput_railcar.h"
#include "drake/automotive/mobil_planner.h"
#include "drake/automotive/pure_pursuit_controller.h"
#include "drake/automotive/simple_car.h"
#include "drake/automotive/trajectory_car.h"
#include "drake/common/drake_copyable.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/rendering/pose_aggregator.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

namespace drake {
namespace automotive {

/// AutomotiveSimulator is a helper class for constructing and running
/// automotive-related simulations.
///
/// @tparam T must be a valid Eigen ScalarType.
///
/// Instantiated templates for the following ScalarTypes are provided:
///
/// - double
///
/// They are already available to link against in the containing library.
template <typename T>
class AutomotiveSimulator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AutomotiveSimulator)

  /// The default constructor that configures this object with a default-
  /// constructed DrakeLcm instance.
  AutomotiveSimulator();

  /// A constructor that configures this object to use a DrakeLcmInterface
  /// instance. If nullptr, no visualization is produced.  Do NOT call
  /// StartRecieveThread on this lcm object.
  explicit AutomotiveSimulator(std::unique_ptr<lcm::DrakeLcmInterface> lcm);

  ~AutomotiveSimulator();

  /// Returns the LCM object used by this AutomotiveSimulator.
  lcm::DrakeLcmInterface* get_lcm();

  /// Returns the DiagramBuilder.
  /// @pre Start() has NOT been called.
  systems::DiagramBuilder<T>* get_builder();

  /// Adds a SimpleCar to this simulation visualized as a Toyota Prius. This
  /// includes its DrivingCommand LCM input.
  ///
  /// @pre Start() has NOT been called.
  ///
  /// @param name The car's name, which must be unique among all cars. Otherwise
  /// a std::runtime_error will be thrown.
  ///
  /// @param channel_name The SimpleCar will subscribe to an LCM channel of this
  /// name to receive commands.  If empty, then the input port is fixed to a
  /// DrivingCommand created using its default constructor.
  ///
  /// @param initial_state The SimpleCar's initial state.
  ///
  /// @return The ID of the car that was just added to the simulation.
  int AddPriusSimpleCar(
      const std::string& name, const std::string& channel_name = std::string(),
      const SimpleCarState<T>& initial_state = SimpleCarState<T>());

  /// Adds a SimpleCar to this simulation controlled by a MOBIL planner coupled
  /// with a PurePursuitController to perform lateral control of the vehicle,
  /// along with an IDM longitudinal controller.  The car is visualized as a
  /// Toyota Prius.
  ///
  /// @pre Start() has NOT been called.
  ///
  /// @pre SetRoadGeometry() was called. Otherwise, a std::runtime_error will be
  /// thrown.
  ///
  /// @param name The car's name, which must be unique among all cars.
  /// Otherwise a std::runtime_error will be thrown.
  ///
  /// @param initial_with_s Initial travel direction in the lane. (See
  /// MobilPlanner documentation.)
  ///
  /// @param path_or_branches If ScanStrategy::kBranches, performs IDM
  /// computations using vehicles detected in confluent branches; if
  /// ScanStrategy::kPath, limits to vehicles on the default path.
  ///
  /// @param road_position_strategy Determines whether or not to memorize
  /// RoadPosition. See `calc_ongoing_road_position.h`.
  ///
  /// @param period_sec The update period to use if road_position_strategy ==
  /// RoadPositionStrategy::kCache.
  ///
  /// @param initial_state The SimpleCar's initial state.
  ///
  /// @return The ID of the car that was just added to the simulation.
  int AddMobilControlledSimpleCar(
      const std::string& name, bool initial_with_s,
      ScanStrategy path_or_branches,
      RoadPositionStrategy road_position_strategy, double period_sec,
      const SimpleCarState<T>& initial_state = SimpleCarState<T>());

  /// Adds a TrajectoryCar to this simulation visualized as a Toyota Prius.
  ///
  /// @pre Start() has NOT been called.
  ///
  /// @param name The car's name, which must be unique among all cars. Otherwise
  /// a std::runtime_error will be thrown.
  ///
  /// @param curve See documentation of TrajectoryCar::TrajectoryCar.
  ///
  /// @param speed See documentation of TrajectoryCar::TrajectoryCar.
  ///
  /// @param start_time See documentation of TrajectoryCar::TrajectoryCar.
  ///
  /// @return The ID of the car that was just added to the simulation.
  int AddPriusTrajectoryCar(const std::string& name,
                            const Curve2<double>& curve, double speed,
                            double start_time);

  /// Adds a lane-following SimpleCar with IdmController and
  /// PurePursuitController to this simulation that takes as input a constant
  /// source that contains the @p goal_lane as the destination lane for the car.
  /// The car is visualized as a Toyota Prius. This includes its
  /// EulerFloatingJoint output.
  ///
  /// @pre Start() has NOT been called.
  ///
  /// @param name The car's name, which must be unique among all cars. Otherwise
  /// a std::runtime_error will be thrown.
  ///
  /// @param initial_with_s Initial travel direction in the lane. True means
  /// that the car is initially oriented in the positive-s direction.
  ///
  /// @param initial_state The SimpleCar's initial state.
  ///
  /// @param goal_lane The goal lane for the car. If goal_lane is nullptr or not
  /// a member of the road supplied via SetRoadGeometry(), a std::runtime_error
  /// will be thrown.
  ///
  /// @param path_or_branches If ScanStrategy::kBranches, performs IDM
  /// computations using vehicles detected in confluent branches; if
  /// ScanStrategy::kPath, limits to vehicles on the default path.
  ///
  /// @param road_position_strategy Determines whether or not to memorize
  /// RoadPosition. See `calc_ongoing_road_position.h`.
  ///
  /// @param period_sec The update period to use if road_position_strategy ==
  /// RoadPositionStrategy::kCache.
  ///
  /// @return The ID of the car that was just added to the simulation.
  int AddIdmControlledCar(const std::string& name,
                          bool initial_with_s,
                          const SimpleCarState<T>& initial_state,
                          const maliput::api::Lane* goal_lane,
                          ScanStrategy path_or_branches,
                          RoadPositionStrategy road_position_strategy,
                          double period_sec);

  /// Adds a MaliputRailcar to this simulation visualized as a Toyota Prius.
  ///
  /// @pre Start() has NOT been called.
  ///
  /// @pre SetRoadGeometry() was called. Otherwise, a std::runtime_error will be
  /// thrown.
  ///
  /// @param name The car's name, which must be unique among all cars. Otherwise
  /// a std::runtime_error will be thrown.
  ///
  /// @param initial_lane_direction The MaliputRailcar's initial lane and
  /// direction on the lane. The lane in this parameter must be part of the
  /// maliput::api::RoadGeometry that is added via SetRoadGeometry(). Otherwise
  /// a std::runtime_error will be thrown.
  ///
  /// @param params The MaliputRailcar's parameters. This is an optional
  /// parameter. Defaults are used if this parameter is not provided.
  ///
  /// @param initial_state The MaliputRailcar's initial state. This is an
  /// optional parameter. Defaults are used if this parameter is not provided.
  ///
  /// @return The ID of the car that was just added to the simulation.
  int AddPriusMaliputRailcar(
      const std::string& name, const LaneDirection& initial_lane_direction,
      const MaliputRailcarParams<T>& params = MaliputRailcarParams<T>(),
      const MaliputRailcarState<T>& initial_state = MaliputRailcarState<T>());

  /// Adds a MaliputRailcar to this simulation visualized as a Toyota Prius that
  /// is controlled via an IdmController.
  ///
  /// @pre Start() has NOT been called.
  ///
  /// @pre SetRoadGeometry() was called. Otherwise, a std::runtime_error will be
  /// thrown.
  ///
  /// @param name The car's name, which must be unique among all cars. Otherwise
  /// a std::runtime_error will be thrown.
  ///
  /// @param initial_lane_direction The MaliputRailcar's initial lane and
  /// direction on the lane. The lane in this parameter must be part of the
  /// maliput::api::RoadGeometry that is added via SetRoadGeometry(). Otherwise
  /// a std::runtime_error will be thrown.
  ///
  /// @param path_or_branches If ScanStrategy::kBranches, performs IDM
  /// computations using vehicles detected in confluent branches; if
  /// ScanStrategy::kPath, limits to vehicles on the default path.
  ///
  /// @param road_position_strategy Determines whether or not to memorize
  /// RoadPosition. See `calc_ongoing_road_position.h`.
  ///
  /// @param period_sec The update period to use if road_position_strategy ==
  /// RoadPositionStrategy::kCache.
  ///
  /// @param params The MaliputRailcar's parameters. This is an optional
  /// parameter. Defaults are used if this parameter is not provided.
  ///
  /// @param initial_state The MaliputRailcar's initial state. This is an
  /// optional parameter. Defaults are used if this parameter is not provided.
  ///
  /// @return The ID of the car that was just added to the simulation.
  int AddIdmControlledPriusMaliputRailcar(
      const std::string& name, const LaneDirection& initial_lane_direction,
      ScanStrategy path_or_branches,
      RoadPositionStrategy road_position_strategy, double period_sec,
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
  /// @pre Start() has NOT been called.
  const maliput::api::RoadGeometry* SetRoadGeometry(
      std::unique_ptr<const maliput::api::RoadGeometry> road);

  /// Finds and returns a pointer to a lane with the specified name. This method
  /// throws a std::runtime_error if no such lane exists.
  ///
  /// @pre SetRoadGeometry() was called.
  ///
  const maliput::api::Lane* FindLane(const std::string& name) const;

  /// Returns the System whose name matches @p name.
  /// @throws std::exception if no such system has been added, or multiple
  /// such systems have been added.
  //
  /// This is the builder variant of the method.  It can only be used prior to
  /// Start() being called.
  ///
  /// @pre Start() has NOT been called.
  systems::System<T>& GetBuilderSystemByName(std::string name);

  /// Returns the System whose name matches @p name.
  /// @throws std::exception if no such system has been added, or multiple such
  /// systems have been added.
  ///
  /// This is the diagram variant of the method, which can only be used after
  /// Start() is called.
  ///
  /// @pre Start() has been called.
  const systems::System<T>& GetDiagramSystemByName(std::string name) const;

  /// Builds the Diagram.  No further changes to the diagram may occur after
  /// this has been called.
  ///
  /// @pre Build() and BuildandInitialize() have NOT been called.
  void Build();

  /// Builds the Diagram and initializes the Diagram Context to the predefined
  /// initial states.
  ///
  /// @pre Build() and BuildandInitialize() have NOT been called.
  void BuildAndInitialize(
      std::unique_ptr<systems::Context<double>> initial_context = nullptr);

  /// Returns the System containing the entire AutomotiveSimulator diagram.
  ///
  /// @pre either Build() or BuildandInitialize() have been called.
  const systems::System<T>& GetDiagram() const {
    DRAKE_DEMAND(diagram_ != nullptr);
    return *diagram_;
  }

  /// Calls BuildAndInitialize() on the diagram (if it has not been build
  /// already) and initializes the Simulator.  No further changes to the diagram
  /// may occur after this has been called.
  ///
  /// @pre Start() has NOT been called.
  ///
  /// @param target_realtime_rate This value is passed to
  /// systems::Simulator::set_target_realtime_rate().
  //
  // TODO(jwnimmer-tri) Perhaps our class should be AutomotiveSimulatorBuilder?
  // Port a few more demo programs, then decide what looks best.
  void Start(double target_realtime_rate = 0.0,
             std::unique_ptr<systems::Context<double>>
             initial_context = nullptr);

  /// Returns whether the automotive simulator has started.
  bool has_started() const { return simulator_ != nullptr; }

  /// Advances simulated time by the given @p time_step increment in seconds.
  void StepBy(const T& time_step);

  /// Returns the current poses of all vehicles in the simulation.
  ///
  /// @pre Start() has been called.
  systems::rendering::PoseBundle<T> GetCurrentPoses() const;

 private:
  int allocate_vehicle_number();

  // Verifies that the provided `name` of a car is unique among all cars that
  // have been added to the `AutomotiveSimulator`. Throws a std::runtime_error
  // if it is not unique meaning a car of the same name was already added.
  void CheckNameUniqueness(const std::string& name);

  // Connects the provided pose and velocity output ports of a vehicle model to
  // the PoseAggregator and adds Prius geometry for visualizing the vehicle.
  void ConnectCarOutputsAndPriusVis(int id,
    const systems::OutputPort<T>& pose_output,
    const systems::OutputPort<T>& velocity_output);

  // Adds an LCM publisher for the given @p system.
  // @pre Start() has NOT been called.
  void AddPublisher(const MaliputRailcar<T>& system, int vehicle_number);

  // Adds an LCM publisher for the given @p system.
  // @pre Start() has NOT been called.
  void AddPublisher(const SimpleCar<T>& system, int vehicle_number);

  // Adds an LCM publisher for the given @p system.
  // @pre Start() has NOT been called.
  void AddPublisher(const TrajectoryCar<T>& system, int vehicle_number);

  // Generates the visualization mesh of the road network and adds it to the
  // SceneGraph.  Member variable `road_` must be set prior to calling this
  // method.  This method is a no-op if visualization is disabled.
  void AddRoadNetworkToSceneGraph();

  void InitializeTrajectoryCars();
  void InitializeSimpleCars();
  void InitializeMaliputRailcars();

  // For both building and simulation.
  std::unique_ptr<lcm::DrakeLcmInterface> owned_lcm_{};
  lcm::DrakeLcmInterface* lcm_{};
  std::unique_ptr<const maliput::api::RoadGeometry> road_{};

  // === Start for building. ===

  std::unique_ptr<systems::DiagramBuilder<T>> builder_{
      std::make_unique<systems::DiagramBuilder<T>>()};

  // Holds the desired initial states of each TrajectoryCar. It is used to
  // initialize the simulation's diagram's state.
  std::map<const TrajectoryCar<T>*, TrajectoryCarState<T>>
      trajectory_car_initial_states_;

  // Holds the desired initial states of each SimpleCar. It is used to
  // initialize the simulation's diagram's state.
  std::map<const SimpleCar<T>*, SimpleCarState<T>> simple_car_initial_states_;

  // Holds the desired initial states of each MaliputRailcar. It is used to
  // initialize the simulation's diagram's state.
  std::map<const MaliputRailcar<T>*,
           std::pair<MaliputRailcarParams<T>, MaliputRailcarState<T>>>
      railcar_configs_;

  // The output port of the Diagram that contains pose bundle information.
  int pose_bundle_output_port_{};

  // === End for building. ===

  systems::rendering::PoseAggregator<T>* aggregator_{};
  geometry::SceneGraph<T>* scene_graph_{};

  int next_vehicle_number_{0};

  // Maps a vehicle id to a pointer to the system that implements the vehicle.
  std::map<int, systems::System<T>*> vehicles_;

  // For simulation.
  std::unique_ptr<systems::Diagram<T>> diagram_{};
  std::unique_ptr<systems::Simulator<T>> simulator_{};
};

}  // namespace automotive
}  // namespace drake
