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

If you experience any problems with or have questions about Drake, please
[ask for help](/getting_help.html).

Drake binary releases incorporate a pre-compiled version of
[SNOPT](https://ccom.ucsd.edu/~optimizers/solvers/snopt/) as part of the
[Mathematical Program toolbox](https://drake.mit.edu/doxygen_cxx/group__solvers.html).
Thanks to Philip E. Gill and Elizabeth Wong for their kind support.

Drake's binary releases do not support the Gurobi solver. To use
Gurobi, you will need to build Drake from source following the instructions in
[Source Installation](/from_source.html).

## Stable Releases

Binary packages of Drake for Ubuntu 20.04 (Focal), Ubuntu 22.04 (Jammy), and
Mac are available to download as attachments from Drake's GitHub
[releases](https://github.com/RobotLocomotion/drake/releases) page.

The most recent release is
[v1.10.0](https://github.com/RobotLocomotion/drake/releases/tag/v1.10.0):

* [https://github.com/RobotLocomotion/drake/releases/download/v1.10.0/drake-20221116-focal.tar.gz](https://github.com/RobotLocomotion/drake/releases/download/v1.10.0/drake-20221116-focal.tar.gz)
* [https://github.com/RobotLocomotion/drake/releases/download/v1.10.0/drake-20221116-jammy.tar.gz](https://github.com/RobotLocomotion/drake/releases/download/v1.10.0/drake-20221116-jammy.tar.gz)
* [https://github.com/RobotLocomotion/drake/releases/download/v1.10.0/drake-20221116-mac.tar.gz](https://github.com/RobotLocomotion/drake/releases/download/v1.10.0/drake-20221116-mac.tar.gz) (for x86_64)
* [https://github.com/RobotLocomotion/drake/releases/download/v1.10.0/drake-20221116-mac-arm64.tar.gz](https://github.com/RobotLocomotion/drake/releases/download/v1.10.0/drake-20221116-mac-arm64.tar.gz) (for arm64)

Users of macOS must download using a command-line tool such as ``curl`` instead
of using a web browser, to avoid hassles from Gatekeeper checks for malicious
software. For example:

```
curl -fsSLO https://github.com/RobotLocomotion/drake/releases/download/v1.10.0/drake-20221116-mac-arm64.tar.gz
```

### Use as a C++ library

For an example of using a Drake ``*.tar.gz`` image from a CMake project, refer
to the
[drake_cmake_installed](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_cmake_installed)
example.

### Use as a Python library

In most cases we suggest [installation via pip](/pip.html) because that
will be more convenient than manually downloading.  However, if you are running
on macOS arm64 or if you need both
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
python3 -m venv env --system-site-packages
source env/bin/activate
```

Install dependencies within the environment:

```bash
env/share/drake/setup/install_prereqs
````

(On Ubuntu, the script might ask to be run under ``sudo``.)

Refer to [Quickstart](/installation.html#quickstart) for next steps.

## Nightly Releases

Binary packages of Drake for Ubuntu 20.04 (Focal), Ubuntu 22.04 (Jammy), and
Mac are generated nightly and are available to download at:

* [https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-focal.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-focal.tar.gz)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-jammy.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-jammy.tar.gz)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-mac.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-mac.tar.gz) (for x86_64)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-mac-arm64.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-mac-arm64.tar.gz) (for arm64)

Older packages for specific dates are available by replacing ``latest`` with an
8-digit date, e.g., ``20221005`` for October 5th, 2022.

* [https://drake-packages.csail.mit.edu/drake/nightly/drake-20221005-focal.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-20221005-focal.tar.gz)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-20221005-jammy.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-20221005-jammy.tar.gz)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-20221005-mac.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-20221005-mac.tar.gz) (for x86_64)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-20221005-mac-arm64.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-20221005-mac-arm64.tar.gz) (for arm64)

Nightly archives are retained for 56 days from their date of creation.

The installation instructions are identical to stable releases as shown above.

As with stable releases, users of macOS must download using a command-line tool
such as ``curl`` instead of using a web browser, to avoid hassles from
Gatekeeper checks for malicious software.
