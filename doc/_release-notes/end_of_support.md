---
title: End of support releases
supplemental: true
---

Due to limited resources, we cannot offer indefinite support
for older operating systems or third-party libraries.
If you need to use these, you can use an old release of Drake.

# Ubuntu packages

* Ubuntu 20.04 (Focal)
  * The last version with support for Ubuntu 20.04 was
    [v1.26.0](https://github.com/RobotLocomotion/drake/releases/tag/v1.26.0).
* Ubuntu 18.04 (Bionic)
  * The last version with support for Ubuntu 18.04 was
    [v1.1.0](https://github.com/RobotLocomotion/drake/releases/tag/v1.1.0).
* Ubuntu 16.04 (Xenial)
  * The last version with support for Ubuntu 16.04 was
    [v0.11.0](https://github.com/RobotLocomotion/drake/releases/tag/v0.11.0).

# macOS packages

* macOS 14 (Sonoma)
  * The last version with support for macOS 14 was
    [v1.45.0](https://github.com/RobotLocomotion/drake/releases/tag/v1.45.0).
* macOS 13 (Ventura)
  * The last version with support for macOS 13 was
    [v1.33.0](https://github.com/RobotLocomotion/drake/releases/tag/v1.33.0).
* macOS x86_64 architecture
  * The last version with support for macOS running on x86_64 hardware was
    [v1.27.0](https://github.com/RobotLocomotion/drake/releases/tag/v1.27.0).
* macOS 12 (Monterey)
  * The last version with support for macOS 12 was
    [v1.22.0](https://github.com/RobotLocomotion/drake/releases/tag/v1.22.0).
* macOS 11 (Big Sur)
  * The last version with support for macOS 11 was
    [v1.9.0](https://github.com/RobotLocomotion/drake/releases/tag/v1.9.0).
* macOS 10.15 (Catalina)
  * The last version with support for macOS 10.15 was
    [v0.35.0](https://github.com/RobotLocomotion/drake/releases/tag/v0.35.0).
* macOS 10.14 (Mojave)
  * The last version with support for macOS 10.14 was
    [v0.24.0](https://github.com/RobotLocomotion/drake/releases/tag/v0.24.0).
* macOS 10.13 (High Sierra)
  * The last version with support for macOS 10.13 was
    [v0.11.0](https://github.com/RobotLocomotion/drake/releases/tag/v0.11.0).

# Wheel packages

* Python 3.14 (Wheel)
  * On Linux, Drake still supports Python 3.14 wheels.
  * On macOS arm64, Drake still supports Python 3.14 wheels.
* Python 3.13 (Wheel)
  * On Linux, Drake still supports Python 3.13 wheels.
  * On macOS arm64, Drake still supports Python 3.13 wheels.
  * On macOS x86_64, there was never support for Python 3.13 wheels.
* Python 3.12 (Wheel)
  * On Linux, Drake still supports Python 3.12 wheels.
  * On macOS arm64, the last version with support for Python 3.12 wheels was
    [v1.46.0](https://github.com/RobotLocomotion/drake/releases/tag/v1.46.0).
  * On macOS x86_64, the last version with support for Python 3.12 wheels was
    [v1.34.0](https://github.com/RobotLocomotion/drake/releases/tag/v1.34.0).
* Python 3.11 (Wheel)
  * On Linux, Drake still supports Python 3.11 wheels.
  * On macOS arm64, the last version with support for Python 3.11 wheels was
    [v1.40.0](https://github.com/RobotLocomotion/drake/releases/tag/v1.40.0).
  * On macOS x86_64, the last version with support for Python 3.11 wheels was
    [v1.34.0](https://github.com/RobotLocomotion/drake/releases/tag/v1.34.0).
* Python 3.10 (Wheel)
  * On Linux, Drake still supports Python 3.10 wheels.
  * On macOS, the last version with support for Python 3.10 wheels was
    [v1.12.0](https://github.com/RobotLocomotion/drake/releases/tag/v1.12.0).
* Python 3.9 (Wheel)
  * On Linux, the last version with support for Python 3.9 wheels was
    [v1.26.0](https://github.com/RobotLocomotion/drake/releases/tag/v1.26.0).
  * On macOS, the last version with support for Python 3.9 wheels was
    [v1.5.0](https://github.com/RobotLocomotion/drake/releases/tag/v1.5.0).
* Python 3.8 (Wheel)
  * On Linux, the last version with support for Python 3.8 wheels was
    [v1.26.0](https://github.com/RobotLocomotion/drake/releases/tag/v1.26.0).
  * On macOS, Drake never supported Python 3.8 wheels.
* Python 3.7 (Wheel)
  * On Linux, the last version with support for Python 3.7 wheels was
    [v1.1.0](https://github.com/RobotLocomotion/drake/releases/tag/v1.1.0).
  * On macOS, Drake never supported Python 3.7 wheels.
* Python 3.6 (Wheel)
  * On Linux, the last version with support for Python 3.6 wheels was
    [v1.1.0](https://github.com/RobotLocomotion/drake/releases/tag/v1.1.0).
  * On macOS, Drake never supported Python 3.6 wheels.

# MATLAB 2015

Download the appropriate binary release (v0.9.x) for your platform from
<https://github.com/RobotLocomotion/drake/releases>.
Simply extract the archive file into a folder of your choice (mine is called ``drake-distro``).

To view the original MATLAB implementation, you may use the tag
[last_sha_with_original_matlab ](https://github.com/RobotLocomotion/drake/tree/last_sha_with_original_matlab).
Note, however, that the dependencies on this branch are out of date and we do
not expect that you will be able to easily compile/run the code, and do not
provide support for this.

## Running MATLAB examples

To run the MATLAB examples, change directories (in MATLAB) into the ``drake-distro/drake`` folder and at the MATLAB prompt do:

```
addpath_drake
```

Then ``cd`` into the examples directories and try some things out.  Here are a few fun ones to get you started:

* ``runLQR`` in the ``examples/CartPole`` directory
* ``runLQR`` in the ``examples/Quadrotor2D`` directory
* ``RimlessWheelPlant.run()`` in the ``examples/RimlessWheel`` directory
* ``StateMachineControl.run()`` in the ``examples/PlanarMonopodHopper`` directory

Please note that you will have to run `addpath_drake` each time you start MATLAB, or [add it to your startup.m](http://www.mathworks.com/help/matlab/ref/startup.html).

## Linux Specific

The version of the standard C++ libraries that are shipped with the Linux distribution of MATLAB is severely outdated and can cause problems when running mex files that are built against a newer version of the standard.  The typical error message in this case reports `Invalid MEX-Files`.

To work around this issue, the symbolic link for the standard C++ library provided by MATLAB must be redirected to point to a more up-to-date version.

First, make sure that a suitable version of the standard library is installed:

```
sudo apt install g++-4.4
```

Now, the symbolic link in MATLAB must be updated to point to the version that was just installed in `/usr/lib`.  An example for MATLAB R2014a is shown below:

```
cd /usr/local/MATLAB/R2014a/sys/os/glnxa64
sudo rm libstdc++.so.6
sudo ln -s /usr/lib/gcc/x86_64-linux-gnu/4.4/libstdc++.so libstdc++.so.6
```
