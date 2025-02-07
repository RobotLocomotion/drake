---
title: Installation and Quickstart
---

# Previewing

Before installing, you can preview Drake online using our interactive Python
notebooks. See the [Tutorials](/index.html#tutorials) page for details.

# Installation

## Supported Configurations

<!-- The macOS python version below should match what's listed in both the root
CMakeLists.txt and tools/workspace/python/repository.bzl. When any Python
version changes, also be sure to grep the full body of documentation to find
any other citations to it (e.g., in PYTHONPATH). -->

The following table shows the configurations and platforms that Drake
officially supports:

| Operating System ⁽¹⁾               | Architecture | Python ⁽²⁾ | End of life ⁽⁴⁾ |
|------------------------------------|--------------|------------|-----------------|
| Ubuntu 22.04 LTS (Jammy Jellyfish) | x86_64       | 3.10 ⁽³⁾   | March 2026      |
| Ubuntu 24.04 LTS (Noble Numbat)    | x86_64       | 3.12 ⁽³⁾   | March 2028      |
| macOS Sonoma (14)                  | arm64        | 3.12 ⁽³⁾   | October 2025    |
| macOS Sequoia (15)                 | arm64        | 3.12 ⁽³⁾   | October 2026    |

"Official support" means that we have Continuous Integration test coverage to
notice regressions, so if it doesn't work for you then please file a bug report.

Unofficially, Drake is also likely to be compatible with newer versions of
Ubuntu or macOS than what are listed, or with Ubuntu 22.04 running on arm64, or
with other versions of Python. However, these are not supported so if it doesn't
work for you then please file a pull request with the fix, not a bug report.

⁽¹⁾ Drake features that perform image rendering (e.g., camera simulation)
maybe require extra setup. See the
[troubleshooting](/troubleshooting.html#gl-init) page for details.

⁽²⁾ CPython is the only Python implementation supported.
Drake is not tested regularly with Anaconda, so if you are using Anaconda you
may experience compatibility hiccups; when asking for help, be sure to mention
that Conda is involved.

⁽³⁾ The Python version shown in the table is supported for all installation
channels. Additionally, when installing via ``pip``
on Ubuntu Python versions 3.10 through 3.13 (inclusive) are supported and
on macOS Python versions 3.11 through 3.12 (inclusive) are supported.
Refer to [OS Support](/stable.html#os-support) for details on our "end of life"
timeline for changing which Python versions are supported.

⁽⁴⁾ These end-of-life dates are estimates.
Refer to [OS Support](/stable.html#os-support) for details.

Additionally, if you are compiling your own C++ code against Drake's C++ code
and are using Drake's pre-compiled binaries, then you must use the same
compiler as our releases:

| Operating System                   | C/C++ Compiler           | Std   |
|------------------------------------|--------------------------|-------|
| Ubuntu 22.04 LTS (Jammy Jellyfish) | GCC 11                   | C++20 |
| Ubuntu 24.04 LTS (Noble Numbat)    | GCC 13                   | C++20 |
| macOS Sonoma (14)                  | Apple LLVM 16 (Xcode 16) | C++20 |
| macOS Sequoia (15)                 | Apple LLVM 16 (Xcode 16) | C++20 |

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

# Quickstart

For Python, refer to
[Using the Python Bindings](/python_bindings.html#using-the-python-bindings).

For C++ sample projects that show how to import Drake as a CMake external
project, please see our gallery of
[external examples](https://github.com/RobotLocomotion/drake-external-examples),
specifically either the
[example CMake project for apt (deb)](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_cmake_installed_apt)
or the
[example CMake project for tar.gz download](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_cmake_installed).
