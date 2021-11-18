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

| Operating System ⁽⁴⁾             | Architecture | Python  |
|----------------------------------|--------------|---------|
| Ubuntu 18.04 LTS (Bionic Beaver) | x86_64 ⁽¹⁾   | 3.6 ⁽³⁾ |
| Ubuntu 20.04 LTS (Focal Fossa)   | x86_64 ⁽¹⁾   | 3.8 ⁽³⁾ |
| macOS Big Sur (11)               | x86_64 ⁽²⁾   | 3.9 ⁽³⁾ |
| macOS Monterey (12)              | x86_64 ⁽²⁾   | 3.9 ⁽³⁾ |

⁽¹⁾ Drake Ubuntu builds assume support for Intel's AVX2 and FMA instructions,
introduced with the Haswell architecture in 2013 with substantial performance
improvements in the Broadwell architecture in 2014. Drake is compiled with
`-march=broadwell` to exploit these instructions (that also works for Haswell
machines). Drake can be used on older machines if necessary by building from
source with that flag removed.

⁽²⁾ Running Drake under Rosetta 2 emulation on arm64 is not supported. Plans
for any future arm64 support on macOS and/or Ubuntu are discussed in
[issue #13514](https://github.com/RobotLocomotion/drake/issues/13514).

⁽³⁾ CPython is the only Python implementation supported.

⁽⁴⁾ Drake features that perform image rendering (e.g., camera simulation)
require a working display server.  Most personal computers will have this
already built in, but some cloud or docker environments may not.

Additionally, if you compiling your own C++ code against Drake's C++ code
and are using Drake's pre-compiled binaries, then you must use the same
compiler as our releases:

| Operating System                 | C/C++ Compiler                 |
|----------------------------------|--------------------------------|
| Ubuntu 18.04 LTS (Bionic Beaver) | GCC 7.5                        |
| Ubuntu 20.04 LTS (Focal Fossa)   | GCC 9.3                        |
| macOS Big Sur (11)               | Apple LLVM 12.0.0 (Xcode 12.4) |
| macOS Monterey (12)              | Apple LLVM 12.0.0 (Xcode 12.4) |

## Available Versions

Drake publishes stable releases approximately once a month, and also
offers nightly builds on an ongoing basis.

For new users, we recommend using the stable releases.  New releases
will be announced on Drake's GitHub
[releases](https://github.com/RobotLocomotion/drake/releases) page and
documented in Drake's [Release Notes](/release_notes/release_notes.html).

Experienced users who want access to the latest features may use the
nightly builds.

## Choose an Installation Method

The following table shows the installation methods available for Drake's
pre-compiled binaries, per the supported platforms and release versions.

The pip packages only support using Drake via Python.
All other packages support both C++ and/or Python.

|                       | Ubuntu | macOS |
|-----------------------|--------|-------|
| Using pip             | [Stable](/pip.html#stable-releases) ⁽¹⁾ | |
| Using apt (deb)       | [Stable](/apt.html#stable-releases) | |
| Using tar.gz download | [Stable](/from_binary.html#stable-releases) or [Nightly](/from_binary.html#nightly-releases) | [Stable](/from_binary.html#stable-releases) or [Nightly](/from_binary.html#nightly-releases) |
| Using Docker Hub      | [Stable](/docker.html#stable-releases) or [Nightly](/docker.html#nightly-releases) | [Stable](/docker.html#stable-releases) or [Nightly](/docker.html#nightly-releases) |

⁽¹⁾ For the moment, we only support ``pip install drake`` for Ubuntu 18.04 LTS
(Bionic Beaver), not Ubuntu 20.04 LTS (Focal Fossa).  Subscribe to
[#15959](https://github.com/RobotLocomotion/drake/issues/15959) for updates.

Alternatively, you can skip the pre-compiled binaries and
[build Drake from source](/from_source.html).

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
