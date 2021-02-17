---
title: Downstream Testing (Drake as a Dependency)
---

# Introduction

To ensure that Drake enables downstream consumption, there are downstream tests
that show basic usage of Drake as a dependency. Those tests are located in
the [drake-external-examples](https://github.com/RobotLocomotion/drake-external-examples)
repository.

# Continuous Integration

Please see the drake-external-examples [Continuous Integration](https://github.com/RobotLocomotion/drake-external-examples#continuous-integration)
section.

# Local Testing

For CMake, see the
[drake_cmake_installed](https://github.com/RobotLocomotion/drake-external-examples/tree/master/drake_cmake_installed#developer-testing)
example.

For Bazel, see the
[drake_bazel_external](https://github.com/RobotLocomotion/drake-external-examples/tree/master/drake_bazel_external)
example, and note the comment in
[WORKSPACE](https://github.com/RobotLocomotion/drake-external-examples/blob/master/drake_bazel_external/WORKSPACE)
which mentions using something like
[local_repository](https://docs.bazel.build/versions/master/be/workspace.html#local_repository) to consume a local checkout of Drake.
