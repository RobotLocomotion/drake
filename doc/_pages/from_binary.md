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

<div class="note" markdown="1">
Drake's binary releases do not support the Gurobi solver. To use
Gurobi, you will need to build Drake from source following the instructions in
[Source Installation](/from_source.html).
</div>

## Stable Releases

Binary packages of Drake for Ubuntu 22.04 (Jammy), Ubuntu 24.04 (Noble), and
Mac are available to download as attachments from Drake's GitHub
[releases](https://github.com/RobotLocomotion/drake/releases) page.

The most recent release is
[v1.50.0](https://github.com/RobotLocomotion/drake/releases/tag/v1.50.0):

* [https://github.com/RobotLocomotion/drake/releases/download/v1.50.0/drake-1.50.0-jammy.tar.gz](https://github.com/RobotLocomotion/drake/releases/download/v1.50.0/drake-1.50.0-jammy.tar.gz)
* [https://github.com/RobotLocomotion/drake/releases/download/v1.50.0/drake-1.50.0-noble.tar.gz](https://github.com/RobotLocomotion/drake/releases/download/v1.50.0/drake-1.50.0-noble.tar.gz)
* https://github.com/RobotLocomotion/drake/releases/download/v1.50.0/drake-1.50.0-mac-arm64.tar.gz (for arm64)

Users of macOS must download using a command-line tool such as ``curl`` instead
of using a web browser, to avoid hassles from Gatekeeper checks for malicious
software. For example:

```
curl -fsSLO https://github.com/RobotLocomotion/drake/releases/download/v1.50.0/drake-1.50.0-mac-arm64.tar.gz
```

### Use as a C++ library

For an example of using a Drake ``*.tar.gz`` image from a CMake project, refer
to the
[drake_cmake_installed](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_cmake_installed)
example.

If not using the example, be sure to install the dependencies before proceeding
by running the setup script inside the download:

```bash
sudo drake/share/drake/setup/install_prereqs
```

### Use as a Python library

In most cases we suggest [installation via pip](/pip.html) because that
will be more convenient than manually downloading.  However, if you need both
C++ and Python API support, then pip will not work.  This section shows
how to incorporate a manual download into a
[virtual environment](https://packaging.python.org/guides/installing-using-pip-and-virtual-environments/#creating-a-virtual-environment)
directory.  In the example below, we will name that directory ``env``, but you
can choose any name.

Download the binary release ``*.tar.gz`` file, using one of the links above.
In the example below, we'll use ``drake.tar.gz`` to refer to it, but your
download will have a more version-specific filename.

#### Ubuntu

Create and activate the environment:

```bash
mkdir -p env
tar -xvzf drake.tar.gz -C env --strip-components=1
python3 -m venv env --system-site-packages
source env/bin/activate
```

Install dependencies within the environment:

```bash
sudo env/share/drake/setup/install_prereqs
```

Refer to [Quickstart](/installation.html#quickstart) for next steps.

#### macOS

Create the environment:

```bash
mkdir -p env
tar -xvzf drake.tar.gz -C env --strip-components=1
```

Install dependencies within the environment:

```bash
env/share/drake/setup/install_prereqs
```

Activate the environment:

```bash
source env/bin/activate
```

Refer to [Quickstart](/installation.html#quickstart) for next steps.

## Nightly Releases

Binary packages of Drake for Ubuntu 22.04 (Jammy), Ubuntu 24.04 (Noble) ⁽¹⁾,
and Mac are generated nightly and are available to download at:

* [https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-jammy.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-jammy.tar.gz)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-noble.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-noble.tar.gz)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-noble-aarch64.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-noble-aarch64.tar.gz)
* https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-mac-arm64.tar.gz

Older packages for specific dates are available by replacing ``latest``
with date YYYYMMDD preceded by ``0.0.``. For example,

* [https://drake-packages.csail.mit.edu/drake/nightly/drake-0.0.20250301-jammy.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-0.0.20250301-jammy.tar.gz)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-0.0.20250301-noble.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-0.0.20250301-noble.tar.gz)
* [https://drake-packages.csail.mit.edu/drake/nightly/drake-0.0.20250301-noble-aarch64.tar.gz](https://drake-packages.csail.mit.edu/drake/nightly/drake-0.0.20250301-noble-aarch64.tar.gz)
* https://drake-packages.csail.mit.edu/drake/nightly/drake-0.0.20250301-mac-arm64.tar.gz

As with stable releases, users of macOS must download using a command-line tool
such as ``curl`` instead of using a web browser, to avoid hassles from
Gatekeeper checks for malicious software.
See the "Stable Releases" section above for a sample command line.

Nightly archives are retained for 56 days from their date of creation.

The installation instructions are identical to stable releases as shown above.

⁽¹⁾ Drake's support for Ubuntu aarch64 binary packages is currently
experimental. Packages are only available on a nightly basis, not for stable
releases. Additionally, packages will only be available for Ubuntu 24.04
(Noble) and future versions. Follow
[#13514](https://github.com/RobotLocomotion/drake/issues/13514) for updates.
