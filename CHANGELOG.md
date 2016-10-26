[//]: # "This is how you write comments in markdown."

Change Log
==========

Introduction
------------

Drake is undergoing continuous development and cannot yet offer a
stable API.  To help downstream users cope, this document provides
brief summaries of user-facing changes.

Unreleased: changes on master, not yet released
-----------------------------------------------

[//]: # "New functionality or APIs."
### Added

 - [#2602][] Added `DRAKE_ASSERT` and `DRAKE_DEMAND` macros to replace
 built-in `assert()` macro.
 - [#2621][] Added `DRAKE_DEPRECATED` macro for portable deprecation.

[//]: # "Altered functionality or APIs."
### Changed
 - [#3902][] Moved all System 1.0 code to drake/system1/ subdirectory.
 - [#3253][] Moved all MATLAB code to drake/matlab/ subdirectory.
 - [#3605][] Deprecated `KinematicsCache::getNumPositions()` and `KinematicsCache::getNumVelocities()` in favor of `KinematicsCache::get_num_positions()` and `KinematicsCache::get_num_velocities()`.
 - [#3566][] Replaced `number_of_foo()` with `get_num_foo()`.
 - [#3276][] The `measure::execution()` function has been replaced by `MeasureExecutionTime()`.
 - [#3246][] Replaced `RigidBody::hasParent()` with `RigidBody::has_mobilizer_joint()`.
 - [#3191][] All matlab solvers code moved from solvers/ to matlab/solvers/.
 - [#3183][] The gflags library is now a required dependency from the superbuild.
 - [#3168][] Renamed `RigidBodyTree::findAncestorBodies()` to be `RigidBodyTree::FindAncestorBodies()`. Modified method to return the results rather than store them in a parameter.
 - [#3157][] Renamed `RigidBodyTree::findJoint()` to be `RigidBodyTree::FindChildBodyOfJoint()` and `RigidBodyTree::findJointId()` to be `RigidBodyTree::FindIndexOfChildBodyOfJoint()`.
 - [#3115][] Modified SDF parser method names to be style guide compliant and more meaningful.
 - [#3078][] Changed `RigidBodyTree::kWorldLinkName` to be `RigidBodyTree::kWorldName`.
 - [#3056][] Renamed methods that add model instances to `RigidBodyTree` and `RigidBodySystem`.
 - [#3049][] Changed `AddRobotFromURDF*` to be `AddModelInstanceFromURDF*`.
 - [#3003][] Made `RigidBodyFrame` member variables private. Added accessors.
 - [#3010][] All header file names under `solvers` are now spelled with lower case and underscore names.
 - [#2984][] Renamed and moved `Polynomial.h` and `TrigPoly.h` from `drake/util` to `drake/common` and into the `drakeCommon` library.
 - [#2963][] Rename RigidBody::CollisionElement to RigidBodyCollisionElement.
 - [#3003][] Made `RigidBodyFrame` member variables private. Added accessors.
 - [#2997][] Renamed drake/Path.h to drake/common/drake_path.h
 - [#2983][] Renamed namespace `Drake` to be `drake`.
 - [#2923][] Updated member variables of `RigidBodyLoop` and `RigidBodActuator` to conform to style guide.
 - [#2913][] Made `RigidBody::com` and `RigidBody::I` private. Added accessors for them.
 - [#2911][] Made `RigidBody::contact_pts` and `RigidBody::mass` private. Added accessors for them.
 - [#2909][] Made `RigidBody::collision_element_ids` and `RigidBody::collision_element_groups` private. Added accessors for them.
 - [#2908][] Made `RigidBody::visual_elements` private. Renamed accessors based on style guide to be `RigidBody::AddVisualElement()` and `RigidBody::GetVisualElements()`.
 - [#2907][] Made `RigidBody::position_num_start` and `RigidBody::velocity_num_start` private. Renamed accessors to conform to style guide.
 - [#2905][] Made `RigidBody::body_index` private. Added necessary accessors.
 - [#2904][] Made `RigidBody::parent` private. Re-named it to be `RigidBody::parent_`. Added necessary accessors.
 - [#2903][] Made `RigidBody::robotnum` private. Re-named it to be `RigidBody::model_id_`.
 - [#2902][] Made `RigidBody::model_name_` private. Re-named `RigidBody::model_name()` to be `RigidBody::get_model_name()`. Added `RigidBody::set_model_name()`.
 - [#2900][] Made `RigidBody::name_` private. Re-named `RigidBody::name()` to be `RigidBody::get_name()`. Added `RigidBody::set_name()`.
 - [#2666][] Changed `TWIST_SIZE` to `drake::kTwistSize`
 - [#2597][] Changed `RigidBodyTree::findLink()` to be `RigidBodyTree::FindBody()`.
 - [#2426][] Changed `RigidBodyTree::findLinkId()` to be `RigidBodyTree::FindBodyIndex()`. Updated APIs of `RigidBodyTree`, `RigidBody`, `RigidBodyTree`, and `RigidBodyFrame` to support notion of a "model ID" that uniquely identifies a model within a `RigidBodySystem`. This enables the same SDF file to be loaded multiple times into the same `RigidBodySystem`.
 - [#2303][] The following member variables should now be accessed via accessor methods: `RigidBody::linkname`, `RigidBody::model_name`, `RigidBodyTree::num_positions`, and `RigidBodyTree::num_velocities`.
 - [#2325][] The following member variable should now be accessed via an accessor method: `RigidBodyConstraint::robot`.
 - [#1992][] Matlab tests must now be explicitly listed in CMakeLists.
 - [#1970][] The `drake` pod Makefile now includes the `install` action.
 - [#1953][] Replace `valuecheckMatrix()` with `CompareMatrices()`.
 - [#2018][] Fix capitalization of `Constraint` and `OptimizationProblem` APIs to match style guide.
 - [#2415][] Require CMake 3.5 or higher to configure and build.
 - [#2779][] Move some rotation functions from drakeGeometryUtil to drake/math.

[//]: # "Lost functionality or APIs."
### Removed / Deprecated

 - [#3685][] Remove support for the Windows platform.
 - [#2610][] RigidBodyTree::getContactPositions and ::getContactPositionsJac are removed.
 - [#2102][] Macports and Cygwin are no longer supported.
 - [#2067][] `core/Core.h` is removed.
 - [#2039][] Ubuntu 12.04 Trusty is no longer supported.
 - [#2809][] Building the Drake superbuild as a POD is no longer natively supported. Use https://sourceforge.net/p/pods/svn/HEAD/tree/Makefile if you require this functionality.

[//]: # "Smaller bug fixes.  No API changes."
### Fixes

 - [#2027][] Realtime C++ tests no longer fail under ctest.
 - [#2008][] `Drake::simulate(4-arg)` actually compiles again now.
 - [#1990][] Windows builds that include `bullet` are working again.
 - [#1975][] Fix coordinate frame error in rigid body collisions.
 - (Assorted) Non-functional fixes for `cpplint` compliance.

Note that not all changes since v0.9.11 have been captured above.
Only changes since approximately 2016-04-01 are noted.

v0.9.11 (2015-10-08)
--------------------

Changes in version v0.9.11 and before are not provided.

[//]: # "You can use PimpMyChangelog to auto-update this list."
[//]: # "https://github.com/pcreux/pimpmychangelog"
[#1953]: https://github.com/RobotLocomotion/drake/issues/1953
[#1970]: https://github.com/RobotLocomotion/drake/issues/1970
[#1975]: https://github.com/RobotLocomotion/drake/issues/1975
[#1990]: https://github.com/RobotLocomotion/drake/issues/1990
[#1992]: https://github.com/RobotLocomotion/drake/issues/1992
[#2008]: https://github.com/RobotLocomotion/drake/issues/2008
[#2018]: https://github.com/RobotLocomotion/drake/issues/2018
[#2027]: https://github.com/RobotLocomotion/drake/issues/2027
[#2039]: https://github.com/RobotLocomotion/drake/issues/2039
[#2067]: https://github.com/RobotLocomotion/drake/issues/2067
[#2102]: https://github.com/RobotLocomotion/drake/issues/2102
[#2303]: https://github.com/RobotLocomotion/drake/issues/2303
[#2325]: https://github.com/RobotLocomotion/drake/issues/2325
[#2415]: https://github.com/RobotLocomotion/drake/issues/2415
[#2426]: https://github.com/RobotLocomotion/drake/issues/2426
[#2597]: https://github.com/RobotLocomotion/drake/issues/2597
[#2602]: https://github.com/RobotLocomotion/drake/issues/2602
[#2610]: https://github.com/RobotLocomotion/drake/issues/2610
[#2621]: https://github.com/RobotLocomotion/drake/issues/2621
[#2666]: https://github.com/RobotLocomotion/drake/issues/2666
[#2779]: https://github.com/RobotLocomotion/drake/issues/2779
[#2809]: https://github.com/RobotLocomotion/drake/issues/2809
[#2900]: https://github.com/RobotLocomotion/drake/issues/2900
[#2902]: https://github.com/RobotLocomotion/drake/issues/2902
[#2903]: https://github.com/RobotLocomotion/drake/issues/2903
[#2904]: https://github.com/RobotLocomotion/drake/issues/2904
[#2905]: https://github.com/RobotLocomotion/drake/issues/2905
[#2907]: https://github.com/RobotLocomotion/drake/issues/2907
[#2908]: https://github.com/RobotLocomotion/drake/issues/2908
[#2909]: https://github.com/RobotLocomotion/drake/issues/2909
[#2911]: https://github.com/RobotLocomotion/drake/issues/2911
[#2913]: https://github.com/RobotLocomotion/drake/issues/2913
[#2923]: https://github.com/RobotLocomotion/drake/issues/2923
[#2963]: https://github.com/RobotLocomotion/drake/issues/2963
[#2983]: https://github.com/RobotLocomotion/drake/issues/2983
[#2984]: https://github.com/RobotLocomotion/drake/issues/2984
[#2997]: https://github.com/RobotLocomotion/drake/issues/2997
[#3003]: https://github.com/RobotLocomotion/drake/issues/3003
[#3010]: https://github.com/RobotLocomotion/drake/issues/3010
[#3049]: https://github.com/RobotLocomotion/drake/issues/3049
[#3056]: https://github.com/RobotLocomotion/drake/issues/3056
[#3078]: https://github.com/RobotLocomotion/drake/issues/3078
[#3115]: https://github.com/RobotLocomotion/drake/issues/3115
[#3157]: https://github.com/RobotLocomotion/drake/issues/3157
[#3168]: https://github.com/RobotLocomotion/drake/issues/3168
[#3183]: https://github.com/RobotLocomotion/drake/issues/3183
[#3191]: https://github.com/RobotLocomotion/drake/issues/3191
[#3246]: https://github.com/RobotLocomotion/drake/issues/3246
[#3253]: https://github.com/RobotLocomotion/drake/issues/3253
[#3276]: https://github.com/RobotLocomotion/drake/issues/3276
[#3566]: https://github.com/RobotLocomotion/drake/issues/3276
[#3685]: https://github.com/RobotLocomotion/drake/issues/3685
[#3902]: https://github.com/RobotLocomotion/drake/issues/3902
