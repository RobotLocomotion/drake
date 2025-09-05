---
title: Source Installation
---

# New Users

For first-time users, we strongly suggest using one of the pre-compiled binaries
described on our [installation](/installation.html) page. This page explains how
to build Drake from source, which is somewhat more challenging.

# Obtaining the Source Code

Drake's source code is available on [GitHub](https://github.com/RobotLocomotion/drake).

In addition to the code on `master`, the source code archives
for each release are published at
`https://github.com/RobotLocomotion/drake/releases/download/v<version>/drake-<version>-src.tar.gz`
with corresponding .sha256 and .sha512 checksums.

# Supported Configurations

The following table shows the configurations and platforms that Drake
officially supports when building from source:

<!-- The operating system requirements should match those listed in the root
     CMakeLists.txt. -->
<!-- The minimum compiler versions should match those listed in both the root
     CMakeLists.txt and tools/workspace/cc/repository.bzl. -->
<!-- The minimum Python version(s) should match those listed in both the root
     CMakeLists.txt and setup/python/pyproject.toml. -->
<!-- The minimum CMake version across all platforms should match that listed
     in the root CMakeLists.txt, and the version range should match that
     listed in tools/install/libdrake/drake-config.cmake.in (and all
     corresponding tests). -->

| Operating System ⁽¹⁾               | Architecture | Python ⁽²⁾ | Bazel | CMake | C/C++ Compiler ⁽³⁾           | Java       |
|------------------------------------|--------------|------------|-------|-------|------------------------------|------------|
| Ubuntu 22.04 LTS (Jammy Jellyfish) | x86_64       | 3.10       | 8.4   | 3.22  | GCC 11 (default) or Clang 15 | OpenJDK 11 |
| Ubuntu 24.04 LTS (Noble Numbat)    | x86_64       | 3.12       | 8.4   | 3.28  | GCC 13 (default) or Clang 19 | OpenJDK 21 |
| macOS Sonoma (14)                  | arm64        | 3.13       | 8.4   | 4.1   | Apple LLVM 16 (Xcode 16.2)   | OpenJDK 23 |
| macOS Sequoia (15)                 | arm64        | 3.13       | 8.4   | 4.1   | Apple LLVM 17 (Xcode 16.4)   | OpenJDK 23 |

"Official support" means that we have Continuous Integration test coverage to
notice regressions, so if it doesn't work for you then please file a bug report.

Unofficially, Drake is also likely to be compatible with newer versions of
Ubuntu or macOS than what are listed, or with Ubuntu running on arm64, or
with other versions of Python or Java. However, these are not supported
so if it doesn't work for you then please file a pull request with the fix,
not a bug report.

All else being equal, we would recommend developers use Ubuntu 24.04 (Noble).

⁽¹⁾ Drake features that perform image rendering (e.g., camera simulation)
maybe require extra setup. See the
[troubleshooting](/troubleshooting.html#gl-init) page for details.

⁽²⁾ CPython is the only Python implementation supported.

⁽³⁾ Drake requires a compiler running in C++20 (or greater) mode.

# Building with CMake

Drake's build rules are defined using Bazel `BUILD` files, but we provide a
CMake wrapper for installing Drake. While this compiles and installs Drake by
invoking Bazel under the hood, it does so according to CMake conventions
and using the options provided via CMake.

For sample projects that show how to import Drake as a CMake external project
(either by building Drake from source, or by downloading a pre-compiled Drake
release) please see our gallery of
[external examples](https://github.com/RobotLocomotion/drake-external-examples).

Otherwise, you can run a build from source by hand like this:

```bash
# Get the sources.
git clone --filter=blob:none https://github.com/RobotLocomotion/drake.git

# Install the build dependencies.
drake/setup/install_prereqs

# Build and install using standard CMake commands.
mkdir drake-build
cd drake-build
cmake ../drake
make install
```

To change the build options, you can run one of the standard CMake GUIs (e.g.,
`ccmake` or `cmake-gui`) or specify command-line options with `-D` to `cmake`.

Important note: when compiling Drake with Clang 17 or newer on Linux, you must
add `-fno-assume-unique-vtables` to your project's `CMAKE_CXX_FLAGS`, or else
Drake's use of run-time type information and dynamic casts will not work correctly.

## Native CMake Options Supported by Drake

A selection of
[CMake variables](https://cmake.org/cmake/help/latest/manual/cmake-variables.7.html)
can be specified by the user to be parsed by Drake's CMake and passed to the
Bazel build.

* [`CMAKE_BUILD_TYPE`](https://cmake.org/cmake/help/latest/variable/CMAKE_BUILD_TYPE.html)
* [`CMAKE_(C|CXX)_COMPILER`](https://cmake.org/cmake/help/latest/variable/CMAKE_LANG_COMPILER.html)
* [`CMAKE_INSTALL_PREFIX`](https://cmake.org/cmake/help/latest/variable/CMAKE_INSTALL_PREFIX.html)

Building and installing Drake also requires a working installation of Python.
When `Python_EXECUTABLE` is specified, it uses the given path to the Python
interpreter. Otherwise, it uses `find_package(Python)` to find the Python
version supported by Drake on the host platform (falling back to finding any
Python version at all if needed). See
[`FindPython`](https://cmake.org/cmake/help/latest/module/FindPython.html)
for further details.

## Drake-specific CMake Options

Drake also defines a number of CMake options to control different facets of
the build.

Adjusting open-source dependencies:

* `WITH_USER_EIGEN` (default `ON`). When `ON`, uses `find_package(Eigen3)`
  to locate a user-provided `Eigen3::Eigen` library
  instead of hard-coding to the operating system version.
* `WITH_USER_FMT` (default `ON`). When `ON`, uses `find_package(fmt)`
  to locate a user-provided `fmt::fmt` library
  instead of hard-coding to the operating system version.
* `WITH_USER_SPDLOG` (default `ON`). When `ON`, uses `find_package(spdlog)`
  to locate a user-provided `spdlog::spdlog` library
  instead of hard-coding to the operating system version.
  * When `ON`, `WITH_USER_FMT` must also be `ON`.
* `WITH_USER_BLAS` (default `ON`). When `ON`, uses `FindBlas()` to locate a
  user-provided `BLAS::BLAS` library instead of building from source.
  * This option is not available on macOS.
* `WITH_USER_LAPACK` (default `ON`). When `ON`, uses `FindLapack()` to locate a
  user-provided `LAPACK::LAPACK` library instead of building from source.
  * This option is not available on macOS.
  * When `ON`, `WITH_USER_BLAS` must also be `ON`.
* `WITH_USER_ZLIB` (default `ON`). When `ON`, uses `find_package(ZLIB)` to
  locate a user-provided `ZLIB::ZLIB` library instead of building from source.
  * Caveat: on macOS, for now this hardcodes `-lz`
    instead of calling `find_package`.
* `WITH_CLARABEL` (default `ON`). When `ON`, enables the `ClarabelSolver`
  in the build.
* `WITH_CLP` (default `ON`). When `ON`, enables the `ClpSolver` in the build.
* `WITH_CSDP` (default `ON`). When `ON`, enables the `CsdpSolver` in the build.
* `WITH_IPOPT` (default `ON`). When `ON`, enables the `IpoptSolver` in the build.
* `WITH_NLOPT` (default `ON`). When `ON`, enables the `NloptSolver` in the build.
* `WITH_OSQP` (default `ON`). When `ON`, enables the `OsqpSolver` in the build.
* `WITH_SCS` (default `ON`). When `ON`, enables the `ScsSolver` in the build.

Adjusting closed-source (commercial) software dependencies:

* `WITH_GUROBI` (default `OFF`).
  When `ON`, enables the `GurobiSolver` in the build.
  * When enabled, you must download and install Gurobi 10.0 yourself prior to
    running Drake's CMake configure script; Drake does not automatically
    download Gurobi. If Gurobi is not installed to its standard location, you
    must also `export GUROBI_HOME=${...GUROBI_UNZIP_PATH...}/linux64`
    in your terminal so that `find_package(Gurobi)` will be able to find it.
* `WITH_MOSEK` (default `OFF`).
  When `ON`, enables the `MosekSolver` in the build.
  * When enabled, Drake automatically downloads the MOSEK™ software from
    `mosek.com` and installs it as part of the Drake build. The selected
    version is hard-coded in Drake and cannot be configured.
* `WITH_SNOPT` (default `OFF`).
  When `ON`, enables the `SnoptSolver` in the build.
  * This option is mutally exclusive with `WITH_ROBOTLOCOMOTION_SNOPT`.
* `SNOPT_PATH` (no default). When `WITH_SNOPT` is `ON`,
  this must be set to a SNOPT source code archive path
  (e.g., `/home/user/Downloads/snopt7.4.tar.gz`) with
  SNOPT version 7.4 (recommended) or version 7.6.
  * Drake does not support using a SNOPT binary release (i.e., shared library);
    it requires a source archive (i.e., the Fortran code).
* `WITH_ROBOTLOCOMOTION_SNOPT` (default `OFF`).
  When `ON`, enables the `SnoptSolver` in the build,
  using a hard-coded and access-controlled download of SNOPT.
  * This option is only valid for MIT- or TRI-affiliated Drake developers.
  * This option is mutally exclusive with `WITH_SNOPT`.

Adjusting installation methods (advanced):

* `INSTALL_NAME_TOOL`. When specified, uses the path to the
  `install_name_tool` program.
  * This option is only available on macOS.
* `INSTALL_STRIP_TOOL`. When specified, uses the path to the `strip` program.

## CMake Caveats

Note that a concurrency limit passed to `make` (e.g., `make -j 2`) for a Drake
build has almost no effect. You might need to add a bazel configuration dotfile
to your home directory if your build is running out of memory. See the
[troubleshooting](/troubleshooting.html#build-oom) page for details.

Be aware that repeatedly running `make install` will install the recompiled
version of Drake *on top of* the prior version. This will lead to disaster
unless the set of installed filenames is exactly the same (because old files
will be hanging around, e.g., polluting your PYTHONPATH). It is safe if you are
merely tweaking a source code file and repeatedly installing, without any
changes to the build system. For any kind of larger change
(e.g., upgrading to a newer Drake), we strongly advise that you delete the
prior tree (within the `install` sub-directory) before running `make`.

## Running the Python Bindings after a CMake Install

To run the installed copy of `pydrake`, you will also need to have your
``PYTHONPATH`` configured correctly.

*Ubuntu 22.04 (Jammy):*

```bash
cd drake-build
export PYTHONPATH=${PWD}/install/lib/python3.10/site-packages:${PYTHONPATH}
```

*Ubuntu 24.04 (Noble):*

```bash
cd drake-build
export PYTHONPATH=${PWD}/install/lib/python3.12/site-packages:${PYTHONPATH}
```

*macOS:*

```bash
cd drake-build
export PYTHONPATH=${PWD}/install/lib/python3.13/site-packages:${PYTHONPATH}
```

# Building with Bazel

If your organization already uses Bazel for its builds, you can build Drake as
a Bazel external. For sample projects that show how to import Drake as a Bazel
external, please see our gallery of
[external examples](https://github.com/RobotLocomotion/drake-external-examples).

* The
[`drake_bazel_external` example](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_bazel_external)
shows how to build Drake from source.
* The
[`drake_bazel_download` example](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_bazel_download)
shows how to download a precompiled release.

When building Drake from source as a Bazel external, we offer flags for
customization. Refer to the comments in
[drake/tools/flags/BUILD.bazel](https://github.com/RobotLocomotion/drake/blob/master/tools/flags/BUILD.bazel)
for details. The `drake_bazel_external` example demonstrates a few of the flags.
If you enable any of proprietary solvers flags, then you must first install
the solver and set environment variables per the
[Proprietary Solvers](/bazel.html#proprietary-solvers) instructions.

There is no way to install Drake from Bazel. To install Drake, use CMake
(see above).

# Making Changes to Drake

Drake developers use Bazel for development.
Refer to our [Bazel instructions](/bazel.html) for details.
