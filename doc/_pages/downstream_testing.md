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
[drake_cmake_installed](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_cmake_installed#developer-testing)
example.

For Bazel, see the
[drake_bazel_external](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_bazel_external)
example, and note the comments in:

* the [README](https://github.com/RobotLocomotion/drake-external-examples/blob/main/drake_bazel_external#using-a-local-checkout-of-Drake),
which mentions using
[`--override-module`](https://bazel.build/reference/command-line-reference#flag--override_module) to consume a local checkout of Drake
* [`MODULE.bazel`](https://github.com/RobotLocomotion/drake-external-examples/blob/main/drake_bazel_external/MODULE.bazel),
which can be modified to use a particular revision (commit or release) of Drake
