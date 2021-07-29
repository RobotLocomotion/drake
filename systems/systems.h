
/// @addtogroup systems
/// @{
/// @brief Drake uses a Simulink-inspired description of dynamical systems.
///
/// Includes basic building blocks (adders, integrators, delays, etc),
/// physics models of mechanical systems, and a growing list of sensors,
/// actuators, controllers, planners, estimators.
///
/// All dynamical systems derive from the drake::systems::System base class, and
/// must explicitly declare all drake::systems::State,
/// drake::systems::Parameters, and noise/disturbances inputs. The
/// drake::systems::Diagram class permits modeling complex systems from
/// libraries of parts.
///
/// For an introduction to using systems in python, see the dynamical_systems
/// tutorial.  For a "Hello, World!" example of writing a dynamical system in
/// C++, see simple_continuous_time_system.cc and/or
/// simple_discrete_time_system.cc.
///
/// @}

// Define systems here so that we can control the order in which they appear
/// @addtogroup systems
/// @{
///   @defgroup primitive_systems Primitives
///   @defgroup control_systems Controllers
///   @defgroup estimator_systems Estimators
///   @defgroup sensor_systems Sensors
///   @defgroup manipulation_systems Manipulation
///   @defgroup message_passing Message Passing
///   @defgroup multibody_systems Multibody Systems
///   @defgroup perception_systems Perception
///   @defgroup discrete_systems Discrete Systems
///   @defgroup stochastic_systems Stochastic Systems
///   @defgroup visualization Visualization
///   @defgroup example_systems Examples
/// @}

/// @addtogroup primitive_systems
/// @{
///   @brief General-purpose Systems such as Gain, Multiplexer, Integrator,
/// and LinearSystem.
/// @}

/// @addtogroup control_systems
/// @{
/// Implementations of controllers that operate as Systems in a block diagram.
/// Algorithms that synthesize controllers are located in @ref control.
/// @}

/// @addtogroup estimator_systems
/// @{
/// Implementations of estimators that operate as Systems in a block diagram.
/// Algorithms that synthesize controllers are located in @ref estimation.
/// @}

/// @addtogroup sensor_systems
/// @{
/// Drake provides a variety of capabilities for sensor modeling. Some sensors
/// are offered as Systems in a block diagram. These sensors are listed in the
/// following section as classes. On the other hand, Drake has some internal
/// functionalities, which can be used to model a sensor. The
/// **Force/Torque Sensor** falls into this catergory.
///
/// A **Force/Torque Sensor** measures nothing but the reaction force/torque
/// of a @ref drake::multibody::WeldJoint "WeldJoint". The
/// @ref drake::multibody::MultibodyPlant "MultibodyPlant" class provides the
/// @ref drake::multibody::MultibodyPlant::get_reaction_forces_output_port()
/// "get_reaction_forces_output_port()" method that returns the reaction
/// @ref drake::multibody::SpatialForce "SpatialForce" of all the joints. You
/// only need to extract the entries for the joints in which you are interested.
/// To simulate a Force/Torque sensor, you need to either a) define a
/// fixed joint for the force/torque sensor in the SDF/URDF model file or
/// b) add a @ref drake::multibody::WeldJoint "WeldJoint" between two bodies.
/// This joint will then serve as the Force/Torque sensor. You can also
/// refer to *planar_gripper* for an example.
/// @}

/// @addtogroup manipulation_systems
/// @{
/// @brief Systems implementations and related functions that specifically
/// support dexterous manipulation capabilities in robotics.
/// @}

/// @addtogroup message_passing
/// @{
/// @brief Systems for publishing/subscribing to popular message passing
/// ecosystems.
/// @}
// TODO(russt): Add pointers to / recommendations for connecting to ROS.
// TODO(russt): Add ZMQ.

/// @addtogroup multibody_systems
/// @{
/// @brief Systems that relate to, or add functionality to, MultibodyPlant.
/// @}

/// @addtogroup perception_systems
/// @{
/// @brief Systems for dealing with perception data and/or wrapping basic
/// perception algorithms.
/// @}

/// @addtogroup visualization
/// @{
/// @brief Systems for connecting to external visualization tools/GUIs.
///
/// There are also a number of external projects that provide additional
/// visualization hook=ups:
/// <ul>
///   <li> Connections to <a
///        href="https://github.com/rdeits/meshcat">MeshCat</a>
///        are available (via the Python bindings) <a
///        href="https://github.com/RussTedrake/underactuated/tree/master/src/underactuated">here</a>.
///   </li>
///   <li> <a href="https://github.com/RussTedrake/underactuated/tree/master/src/underactuated">
///        A matplotlib visualization for planar rigid body systems</a>.</li>
/// </ul>
/// @}
// TODO(russt): Add pointers to / support for RViz.

/// @addtogroup example_systems
/// @{
/// @brief The examples contain a number of useful System implementations.
/// @}
