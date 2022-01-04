---
title: Installation via Direct Download
---

# Binary Packages

Drake publishes pre-compiled binaries as binary downloads (``*.tar.gz``)
for all supported operating systems.  Refer to
[Supported Configurations](/installation.html#supported-configurations)
for compatibility details.

To learn about other installation methods, refer to
[Installation and Quickstart](/installation.html).

If you experience any problems or questions with Drake, please
[ask for help on Stack Overflow](/getting_help.html).

Drake binary releases incorporate a pre-compiled version of
[SNOPT](https://ccom.ucsd.edu/~optimizers/solvers/snopt/) as part of the
[Mathematical Program toolbox](https://drake.mit.edu/doxygen_cxx/group__solvers.html).
Thanks to Philip E. Gill and Elizabeth Wong for their kind support.

## Stable Releases

Binary packages of Drake for Ubuntu 18.04 (Bionic), Ubuntu 20.04 (Focal) and
Mac are available to download as attachments from Drake's GitHub
[releases](https://github.com/RobotLocomotion/drake/releases) page.

The most recent release is
[v0.37.0](https://github.com/RobotLocomotion/drake/releases/tag/v0.37.0):

* [https://github.com/RobotLocomotion/drake/releases/download/v0.37.0/drake-20211213-bionic.tar.gz](https://github.com/RobotLocomotion/drake/releases/download/v0.37.0/drake-20211213-bionic.tar.gz)
* [https://github.com/RobotLocomotion/drake/releases/download/v0.37.0/drake-20211213-focal.tar.gz](https://github.com/RobotLocomotion/drake/releases/download/v0.37.0/drake-20211213-focal.tar.gz)
* [https://github.com/RobotLocomotion/drake/releases/download/v0.37.0/drake-20211213-mac.tar.gz](https://github.com/RobotLocomotion/drake/releases/download/v0.37.0/drake-20211213-mac.tar.gz)

### Use as a C++ library

For an example of using a Drake ``*.tar.gz`` image from a CMake project, refer
to the
[drake_cmake_installed](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_cmake_installed)
example.

### Use as a Python library

In most cases, we suggest using our [pip releases](/pip.html), because that
will be more convenient than manually downloading.  However, if you need both
C++ and Python API support, then pip will not work.  This section shows
how to incorporate a manual download into a
[virtual environment](https://packaging.python.org/guides/installing-using-pip-and-virtual-environments/#creating-a-virtual-environment)
directory.  In the example below, we will name that directory ``env``, but you
can choose any name.

Download the binary release ``*.tar.gz`` file, using one of the links above.
In the example below, we'll use ``drake.tar.gz`` to refer to it, but your
download will have a more version-specific filename.

Create and activate the envionment:

```bash
mkdir -p env
tar -xvzf drake.tar.gz -C env --strip-components=1
python3 -m virtualenv -p python3 env --system-site-packages
source env/bin/activate
```

Install dependencies within the environment:

```bash
env/share/drake/setup/install_prereqs
````

(On Ubuntu, the script might ask to be run under ``sudo``.)

Refer to [Quickstart](/installation.html#quickstart) for next steps.

## Nightly Releases

Binary packages of Drake for Ubuntu 18.04 (Bionic), Ubuntu 20.04 (Focal) and
Mac are generated nightly and are available to download at:

* [https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-bionic.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-bionic.tar.gz)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-focal.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-focal.tar.gz)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-mac.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-mac.tar.gz)

Older packages for specific dates are available by replacing ``latest`` with an
8-digit date, e.g., ``20200102`` for January 2nd, 2020.

* [https://drake-packages.csail.mit.edu/drake/nightly/drake-YYYYMMDD-bionic.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-YYYYMMDD-bionic.tar.gz)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-YYYYMMDD-focal.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-YYYYMMDD-focal.tar.gz)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-YYYYMMDD-mac.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-YYYYMMDD-mac.tar.gz)

Individual packages are archived two years from their date of creation.

The installation instructions are identical to stable releases as shown above.
