/** @defgroup systems Modeling Dynamical Systems
 * @{
 * @brief Drake uses a Simulink-inspired description of dynamical systems.
 *
 * Includes basic building blocks (adders, integrators, delays, etc),
 * physics models of mechanical systems, and a growing list of sensors,
 * actuators, controllers, planners, estimators.
 *
 * All dynamical systems derive from the System base class, and must
 * explicitly  declare all State, parameters, and noise/disturbances inputs.
 * The Diagram class permits modeling complex systems from libraries of parts.
 * @}
 */
