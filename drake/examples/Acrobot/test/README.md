Acrobot Example Unit Tests
==========================

Acrobot is a relatively simple 2-DOF robot that can be used when testing Drake.
This page contains instructions on how to run its unit tests.

Execution Instructions
----------------------
From within a terminal, execute:

```
$ cd [drake distro]/drake/pod-build
$ ctest -VV -R acrobotURDFDynamicsTest
```