******************************************
Downstream Testing (Drake as a Dependency)
******************************************

Introduction
============

To ensure that Drake enables downstream consumption, there are downstream tests
that show basic usage of Drake as a dependency. Those tests are located in
the `Drake Shambhala <https://github.com/RobotLocomotion/drake-shambhala>`_
repository.

Continuous Integration
======================

Please see the Drake Shambhala `Continuous Integration
<https://github.com/RobotLocomotion/drake-shambhala#continuous-integration>`_
section.

Local Testing
=============

For CMake, see the
`drake_cmake_installed
<https://github.com/RobotLocomotion/drake-shambhala/tree/master/drake_cmake_installed#developer-testing>`_
example.

For Bazel, see the
`drake_bazel_external <https://github.com/RobotLocomotion/drake-shambhala/tree/master/drake_bazel_external>`_
example, and note the comment in
`WORKSPACE <https://github.com/RobotLocomotion/drake-shambhala/blob/master/drake_bazel_external/WORKSPACE>`_
which mentions using something like
`local_repository <https://docs.bazel.build/versions/master/be/workspace.html#local_repository>`_
to consume a local checkout of Drake.
