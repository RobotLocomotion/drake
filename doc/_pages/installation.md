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

The following table shows the configurations that Drake
officially supports:

| Operating System ⁽¹⁾               | Architecture | Python ⁽²⁾ ⁽³⁾ | End of life ⁽⁴⁾ |
|------------------------------------|--------------|----------------|-----------------|
| Ubuntu 22.04 LTS (Jammy Jellyfish) | x86_64       | 3.10           | March 2026      |
| Ubuntu 24.04 LTS (Noble Numbat)    | x86_64       | 3.12           | March 2028      |
| macOS Sequoia (15)                 | arm64        | 3.14           | October 2026    |
| macOS Tahoe (26)                   | arm64        | 3.14           | October 2027    |

"Official support" means that we have Continuous Integration test coverage to
notice regressions, so if it doesn't work for you then please file a bug report.

Unofficially, Drake is also likely to be compatible with newer versions of
Ubuntu or macOS than what are listed, or with Ubuntu 24.04 running on arm64, or
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
on Ubuntu Python versions 3.10 through 3.14 (inclusive) are supported and
on macOS Python versions 3.13 through 3.14 (inclusive) are supported.
Refer to [OS Support](/stable.html#os-support) for details on our "end of life"
timeline for changing which Python versions are supported.

⁽⁴⁾ These end-of-life dates are estimates.
Refer to [OS Support](/stable.html#os-support) for details.

The following table shows the configurations that *must* be used when
compiling your own C++ code against Drake's C++ code using one of
Drake's pre-compiled binaries:

| Operating System                   | C/C++ Compiler             | Std   |
|------------------------------------|----------------------------|-------|
| Ubuntu 22.04 LTS (Jammy Jellyfish) | GCC 11                     | C++20 |
| Ubuntu 24.04 LTS (Noble Numbat)    | GCC 13                     | C++23 |
| macOS Sequoia (15)                 | Apple LLVM 17 (Xcode 26.2) | C++23 |
| macOS Tahoe (26)                   | Apple LLVM 17 (Xcode 26.2) | C++23 |

Any other configuration not listed here will lead to undefined behavior
(as a violation of the C++ One-Definition Rule).

The above information covers the current version of Drake. For historical
details, see [End of support releases](/release_notes/end_of_support.html).

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

The following installation methods are available for Drake's
pre-compiled binaries.
Stable and nightly release versions are available for each method.

* [Pip](/pip.html) (only supports Python)
* [APT](/apt.html) (only supports Ubuntu)
* [Binary (`*.tar.gz`) download](/from_binary.html)
* [Docker Hub](/docker.html)

Alternatively, you can skip the pre-compiled binaries and build Drake
following the instructions in [Source Installation](/from_source.html).

<div class="note" markdown="1">
Drake's binary releases do not support the Gurobi solver.
To use Gurobi, you must build Drake from source.
</div>

# Quickstart

## Python

Refer to [Using the Python Bindings](/python_bindings.html#using-the-python-bindings)
for useful information and getting started.

Additionally, see our gallery of
[external examples](https://github.com/RobotLocomotion/drake-external-examples)
for sample Python projects that show how to use Drake from Python.

* The [`drake_pip` example](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_pip)
shows how to use Drake via pip.
* The [`drake_poetry` example](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_poetry)
shows how to use Drake via poetry.

For further details, refer to the
[Python API Documentation](/pydrake/index.html).

## C++

See our gallery of
[external examples](https://github.com/RobotLocomotion/drake-external-examples)
for sample C++ projects that show how to import Drake as a CMake external.

* The [`drake_cmake_installed_apt` example](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_cmake_installed_apt)
shows how to use Drake via APT installation.
* The [`drake_cmake_installed` example](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_cmake_installed)
shows how to use Drake via binary (`tar.gz`) installation.

For further details, refer to the
[C++ API Documentation](/doxygen_cxx/index.html).
