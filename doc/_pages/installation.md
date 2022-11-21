---
title: Installation and Quickstart
---

# Previewing

Before installing, you can preview Drake online using our interactive Python
notebooks. See the [Tutorials](/index.html#tutorials) page for details.

# Installation

## Supported Configurations

The following table shows the configurations and platforms that Drake
officially supports:

| Operating System ⁽²⁾               | Architecture     | Python ⁽¹⁾ |
|------------------------------------|------------------|------------|
| Ubuntu 20.04 LTS (Focal Fossa)     | x86_64           | 3.8        |
| Ubuntu 22.04 LTS (Jammy Jellyfish) | x86_64           | 3.10       |
| macOS Monterey (12)                | x86_64 or arm64  | 3.10       |

⁽¹⁾ CPython is the only Python implementation supported.
Drake does not support the Python environment supplied by Anaconda. Before
installing or using Drake, please `conda deactivate` (repeatedly, until even
the conda base environment has been deactivated) such that none of the paths
reported `which -a python python3` refer to conda.
Note that Miniconda seems to work fine; it's only Anaconda that has caused
problems for some users.

⁽²⁾ Drake features that perform image rendering (e.g., camera simulation)
require a working display server.  Most personal computers will have this
already built in, but some cloud or docker environments may require extra
setup steps.

Additionally, if you are compiling your own C++ code against Drake's C++ code
and are using Drake's pre-compiled binaries, then you must use the same
compiler as our releases:

| Operating System                   | C/C++ Compiler           | Std   |
|------------------------------------|--------------------------|-------|
| Ubuntu 20.04 LTS (Focal Fossa)     | GCC 9                    | C++17 |
| Ubuntu 22.04 LTS (Jammy Jellyfish) | GCC 11                   | C++20 |
| macOS Monterey (12)                | Apple LLVM 14 (Xcode 14) | C++20 |

## Available Versions

Drake publishes stable releases approximately once a month, and also
offers nightly builds on an ongoing basis.

For new users, we recommend using the stable releases.  New releases
will be announced on Drake's GitHub
[releases](https://github.com/RobotLocomotion/drake/releases) page and
documented in Drake's [Release Notes](/release_notes/release_notes.html).
Refer to our [Drake Stability Guidelines](/stable.html) for our policy
on API changes.

Experienced users who want access to the latest features may use the
nightly builds.

## Choose an Installation Method

The following table shows the installation methods available for Drake's
pre-compiled binaries, per the supported platforms and release versions.

The pip packages only support using Drake via Python.
All other packages support both C++ and/or Python.

|                       | Ubuntu | macOS |
|-----------------------|--------|-------|
| Using pip             | [Stable](/pip.html#stable-releases) or [Nightly](/pip.html#nightly-releases) | [Stable](/pip.html#stable-releases) or [Nightly](/pip.html#nightly-releases) |
| Using apt (deb)       | [Stable](/apt.html#stable-releases) or [Nightly](/apt.html#nightly-releases) | |
| Using tar.gz download | [Stable](/from_binary.html#stable-releases) or [Nightly](/from_binary.html#nightly-releases) | [Stable](/from_binary.html#stable-releases) or [Nightly](/from_binary.html#nightly-releases) |
| Using Docker Hub      | [Stable](/docker.html#stable-releases) or [Nightly](/docker.html#nightly-releases) | [Stable](/docker.html#stable-releases) or [Nightly](/docker.html#nightly-releases) |

Alternatively, you can skip the pre-compiled binaries and build Drake
following the instructions in [Source Installation](/from_source.html).

Drake's binary releases do not support the Gurobi solver.
To use Gurobi, you must build Drake from source.

We're considering adding macOS support for Homebrew, i.e., ``brew install
drake``.  Please upvote or comment on
[#12782](https://github.com/RobotLocomotion/drake/issues/12782)
if you are interested.

# Quickstart

For Python, refer to
[Using the Python Bindings](/python_bindings.html#using-the-python-bindings).

For C++, refer to either the
[example CMake project for apt (deb)](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_cmake_installed_apt)
or the
[example CMake project for tar.gz download](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_cmake_installed).
