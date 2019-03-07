
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
/// For a "Hello, World!" example of writing a dynamical system, see
/// simple_continuous_time_system.cc and/or simple_discrete_time_system.cc.
/// For an example of a system that uses some of the more advanced
/// features, such as typed input, output, state, and parameter vectors,
/// see simple_car.h .
///
/// @}

// Define systems here so that we can control the order in which they appear
/// @addtogroup systems
/// @{
///   @defgroup primitive_systems Primitives
///   @defgroup control_systems Controllers
///   @defgroup estimator_systems Estimators
///   @defgroup sensor_systems Sensors
///   @defgroup automotive_systems Automotive Systems
///   @defgroup manipulation_systems Manipulation
///   @defgroup message_passing Message Passing
///   @defgroup perception_systems Perception
///   @defgroup discrete_systems Discrete Systems
///   @defgroup stochastic_systems Stochastic Systems
///   @defgroup visualization Visualization
///   @defgroup example_systems Examples
///   @defgroup rigid_body_systems (Attic) Rigid-Body Systems
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
/// Models of sensors that operate as Systems in a block diagram.
/// @}

/// @addtogroup manipulation_systems
/// @{
/// @brief Systems implementations that specifically support dexterous
/// manipulation capabilities in robotics.
/// @}

/// @addtogroup message_passing
/// @{
/// @brief Systems for publishing/subscribing to popular message passing
/// ecosystems.
/// @}
// TODO(russt): Add pointers to / recommendations for connecting to ROS.
// TODO(russt): Add ZMQ.

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
// TODO(russt): Add pointers to / support for for RViz.

/// @addtogroup example_systems
/// @{
/// @brief The examples contain a number of useful System implementations.
/// @}
