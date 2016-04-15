Acrobot Example
===============

Acrobot is a relatively simple 2-DOF robot that can be used when testing Drake.

Unit Test Execution Instructions
--------------------------------
From within a terminal, execute:

```
$ cd [drake distro]/drake/pod-build
$ ctest -VV -R acrobotURDFDynamicsTest
```