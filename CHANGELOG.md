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

 - None

[//]: # "Altered functionality or APIs."
### Changed

 - [#1992][] Matlab tests must now be explicitly listed in CMakeLists.
 - [#1970][] The drake pod Makefile now includes the `install` action.
 - [#1953][] Replace valuecheckMatrix() with CompareMatrices().
 
[//]: # "Lost functionality or APIs."
### Removed / Deprecated

 - None

[//]: # "Smaller bug fixes.  No API changes."
### Fixes

 - [#2008][] `Drake::simulate(4-arg)` actually compiles again now.
 - [#1990][] Windows builds that include bullet are working again.
 - [#1985][] runQuadrotorDynamics no longer asserts realtime performance when run under ctest.
 - [#1975][] Fix coordinate frame error in rigid body collisions.
 - (Assorted) Non-functional fixes for cpplint compliance.

Note that not all changes since v0.9.11 have been captured in the above.
Only changes since approximately 2016-04-01 have been noted.

v0.9.11 (2015-10-08)
--------------------

Changes in version v0.9.11 and before are not provided.

[//]: # "You can use PimpMyChangelog to auto-update this list."
[//]: # "https://github.com/pcreux/pimpmychangelog."
[#1953]: https://github.com/RobotLocomotion/drake/issues/1953
[#1970]: https://github.com/RobotLocomotion/drake/issues/1970
[#1975]: https://github.com/RobotLocomotion/drake/issues/1975
[#1985]: https://github.com/RobotLocomotion/drake/issues/1985
[#1990]: https://github.com/RobotLocomotion/drake/issues/1990
[#1992]: https://github.com/RobotLocomotion/drake/issues/1992
[#2008]: https://github.com/RobotLocomotion/drake/issues/2008
