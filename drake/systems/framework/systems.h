
/// @defgroup systems Modeling Dynamical Systems
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
/// @}
