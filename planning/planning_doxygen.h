/** @addtogroup planning
 @{
   A collection of algorithms for finding configurations and/or trajectories of
   dynamical systems.

   Many planning algorithms make heavy use of @ref solvers::MathematicalProgram
   "MathematicalProgram" and the numerous @ref solver_evaluators.  There are
   also some useful classes in @ref manipulation_systems.

   @defgroup planning_kinematics
   @defgroup planning_trajectory
   @defgroup planning_collision_checker
   @defgroup planning_infrastructure
   @defgroup planning_iris
 @}
*/

/** @addtogroup planning_kinematics Inverse kinematics
 @{
   These algorithms help define configurations of dynamical systems based on
   inverse kinematics (IK).
 @}
*/

/** @addtogroup planning_trajectory Trajectories
 @{
   These algorithms compute trajectories based on various criteria.
 @}
*/

/** @addtogroup planning_collision_checker Collision checking
 @{
  The CollisionChecker interface provides a basis for performing distance
  queries on robots in environments for various planning problems (e.g.,
  sampling planning).
 @}
*/

/** @addtogroup planning_infrastructure Convenience classes
 @{
   Simplifications for managing @ref drake::systems::Diagram "Diagrams"
   containing @ref drake::multibody::MultibodyPlant "MultibodyPlant" and
   @ref drake::geometry::SceneGraph "SceneGraph" systems in planning tasks.
 @}
*/

/** @addtogroup planning_iris Iris
 @{
  These algorithms help construct regions of configuration space that are
  collision free.
 @}
*/
