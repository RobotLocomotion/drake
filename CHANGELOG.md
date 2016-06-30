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

 - [#2602][] Added `DRAKE_ASSERT` and `DRAKE_ABORT_UNLESS` macros to replace
 built-in `assert()` macro.
 - [#2621][] Added `DRAKE_DEPRECATED` macro for portable deprecation.

[//]: # "Altered functionality or APIs."
### Changed

 - [#2597][] Changed `RigidBodyTree::findLink()` to be `RigidBodyTree::FindBody()`.
 - [#2426][] Changed `RigidBodyTree::findLinkId()` to be `RigidBodyTree::FindBodyIndex()`. Updated APIs of `RigidBodyTree`, `RigidBody`, `RigidBodyTree`, and `RigidBodyFrame` to support notion of a "model ID" that uniquely identifies a model within a `RigidBodySystem`. This enables the same SDF file to be loaded multiple times into the same `RigidBodySystem`.
 - [#2303][] The following member variables should now be accessed via accessor methods: `RigidBody::linkname`, `RigidBody::model_name`, `RigidBodyTree::num_positions`, and `RigidBodyTree::num_velocities`.
 - [#2325][] The following member variable should now be accessed via an accessor method: `RigidBodyConstraint::robot`.
 - [#1992][] Matlab tests must now be explicitly listed in CMakeLists.
 - [#1970][] The `drake` pod Makefile now includes the `install` action.
 - [#1953][] Replace `valuecheckMatrix()` with `CompareMatrices()`.
 - [#2018][] Fix capitalization of `Constraint` and `OptimizationProblem` APIs to match style guide.
 - [#2415][] Require CMake 3.5 or higher to configure and build.

[//]: # "Lost functionality or APIs."
### Removed / Deprecated

 - [#2610][] RigidBodyTree::getContactPositions and ::getContactPositionsJac are removed.
 - [#2102][] Macports and Cygwin are no longer supported.
 - [#2067][] `core/Core.h` is removed.
 - [#2039][] Ubuntu 12.04 Trusty is no longer supported.

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
[#2102]: https://github.com/RobotLocomotion/drake/issues/2102
[#2067]: https://github.com/RobotLocomotion/drake/issues/2067
[#2039]: https://github.com/RobotLocomotion/drake/issues/2039
[#2027]: https://github.com/RobotLocomotion/drake/issues/2027
[#1953]: https://github.com/RobotLocomotion/drake/issues/1953
[#1970]: https://github.com/RobotLocomotion/drake/issues/1970
[#1975]: https://github.com/RobotLocomotion/drake/issues/1975
[#1990]: https://github.com/RobotLocomotion/drake/issues/1990
[#1992]: https://github.com/RobotLocomotion/drake/issues/1992
[#2008]: https://github.com/RobotLocomotion/drake/issues/2008
[#2018]: https://github.com/RobotLocomotion/drake/issues/2018
[#2303]: https://github.com/RobotLocomotion/drake/issues/2303
[#2325]: https://github.com/RobotLocomotion/drake/issues/2325
[#2426]: https://github.com/RobotLocomotion/drake/issues/2426
[#2597]: https://github.com/RobotLocomotion/drake/issues/2597
[#2602]: https://github.com/RobotLocomotion/drake/issues/2602
[#2621]: https://github.com/RobotLocomotion/drake/issues/2621
