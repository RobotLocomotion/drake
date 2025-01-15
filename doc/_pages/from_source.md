---
title: Source Installation
---

# New Users

For first-time users, we strongly suggest using one of the pre-compiled binaries
described on our [installation](/installation.html) page. This page explains how
to build Drake form source, which is somewhat more challenging.

# Supported Configurations

The following table shows the configurations and platforms that Drake
officially supports when building from source:

<!-- The operating system requirements should match those listed in the root
     CMakeLists.txt. -->
<!-- The minimum compiler versions should match those listed in both the root
     CMakeLists.txt and tools/workspace/cc/repository.bzl. -->
<!-- The minimum Python version(s) should match those listed in both the root
     CMakeLists.txt and setup/python/pyproject.toml. -->

| Operating System ⁽¹⁾               | Architecture | Python ⁽²⁾ | Bazel | CMake | C/C++ Compiler ⁽³⁾           | Java          |
|------------------------------------|--------------|------------|-------|-------|------------------------------|------------|
| Ubuntu 22.04 LTS (Jammy Jellyfish) | x86_64       | 3.10       | 8.0   | 3.22  | GCC 11 (default) or Clang 15 | OpenJDK 11 |
| Ubuntu 24.04 LTS (Noble Numbat)    | x86_64       | 3.12       | 8.0   | 3.28  | GCC 13 (default) or Clang 15 | OpenJDK 21 |
| macOS Sonoma (14)                  | arm64        | 3.12       | 8.0   | 3.31  | Apple LLVM 16 (Xcode 16)     | OpenJDK 23 |
| macOS Sequoia (15)                 | arm64        | 3.12       | 8.0   | 3.31  | Apple LLVM 16 (Xcode 16)     | OpenJDK 23 |

"Official support" means that we have Continuous Integration test coverage to
notice regressions, so if it doesn't work for you then please file a bug report.

Unofficially, Drake is also likely to be compatible with newer versions of
Ubuntu or macOS than what are listed, or with Ubuntu running on arm64, or
with other versions of Python or Java. However, these are not supported so if it
doesn't work for you then please file a pull request with the fix, not a bug
report.

All else being equal, we would recommend developers use Ubuntu 22.04 (Jammy).

⁽¹⁾ Drake features that perform image rendering (e.g., camera simulation)
require a working display server. Most personal computers will have this
already built in, but some cloud or docker environments may require extra
setup steps.

⁽²⁾ CPython is the only Python implementation supported.

⁽³⁾ Drake requires a compiler running in C++20 (or greater) mode.

# Building with CMake

For sample projects that show how to import Drake as a CMake external project
(either by building Drake from source, or by downloading a pre-compiled Drake
release) please see our gallery of
[drake-external-examples](https://github.com/RobotLocomotion/drake-external-examples).

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

## CMake options which are Drake-specific

These options can be set using `-DFOO=bar` on the CMake command line, or in one
of the CMake GUIs.

Adjusting open-source dependencies:

* WITH_USER_EIGEN (default OFF). When ON, uses `find_package(Eigen3)`
  to locate a user-provided `Eigen3::Eigen` library
  instead of hard-coding to the operating system version.
* WITH_USER_FMT (default OFF). When ON, uses `find_package(fmt)`
  to locate a user-provided `fmt::fmt` library
  instead of hard-coding to the operating system version.
* WITH_USER_SPDLOG (default OFF). When ON, uses `find_package(spdlog)`
  to locate a user-provided `spdlog::spdlog` library
  instead of hard-coding to the operating system version.

Adjusting closed-source (commercial) software dependencies:

* WITH_GUROBI (default OFF). When ON, enables the `GurobiSolver` in the build.
  * When enabled, you must download and install Gurobi 10.0 yourself prior to
    running Drake's CMake configure script; Drake does not automatically
    download Gurobi. If Gurobi is not installed to its standard location, you
    must also specify `export GUROBI_HOME=${...GUROBI_UNZIP_PATH...}/linux64`
    in your terminal so that `find_package(Gurobi)` will be able to find it.
* WITH_MOSEK (default OFF). When ON, enables the `MosekSolver` in the build.
  * When enabled, Drake automatically downloads the MOSEK™ software from
    `mosek.com` and installs it as part of the Drake build. The selected
    version is hard-coded in Drake and cannot be configured.
* WITH_SNOPT (default OFF). When ON, enables the `SnoptSolver` in the build.
  * This option is mutally exclusive with `WITH_ROBOTLOCOMOTION_SNOPT`.
* SNOPT_PATH (no default). When `WITH_SNOPT` is ON, this must be set to a SNOPT
  source code archive path (e.g., `/home/user/Downloads/snopt7.4.tar.gz`) with
  SNOPT version 7.4 (recommended) or version 7.6.
  * Drake does not support using a SNOPT binary release (i.e., shared library);
    it requires a source archive (i.e., the Fortran code).
* WITH_ROBOTLOCOMOTION_SNOPT (default OFF). When ON, enables the `SnoptSolver`
  in the build, using a hard-coded and access-controlled download of SNOPT.
  This option is only valid for MIT- or TRI-affiliated Drake developers.
  * This option is mutally exclusive with `WITH_SNOPT`.

## CMake caveats

Note that a concurrency limit passed to `make` (e.g., `make -j 2`) for a Drake
build has almost no effect. You might need to add a bazel configuration dotfile
to your home directory if your build is running out of memory. See the
[troubleshooting](/troubleshooting.html#build-oom) page for details.

Be aware that repeatedly running `make install` will install the recompiled
version of Drake *on top of* the prior version. This will lead to disaster
unless the set of installed filenames is exactly the same (because old files
will be hanging around, e.g., polluting your PYTHONPATH). It is safe if you are
merely tweaking a source code file and repeatedly installing, without any
changes to the build system. For any kind of larger change (e.g., upgrading to a
newer Drake), we strongly advise that you delete the prior tree (within the
`install` sub-directory) before running `make`.

## Running the Python Bindings after a CMake install

To run the installed copy of `pydrake`, you will also need to have your
``PYTHONPATH`` configured correctly.

*Ubuntu 22.04 (Jammy):*

```bash
cd drake-build
export PYTHONPATH=${PWD}/install/lib/python3.10/site-packages:${PYTHONPATH}
```

*Ubuntu 24.04 (Jammy):*

```bash
cd drake-build
export PYTHONPATH=${PWD}/install/lib/python3.12/site-packages:${PYTHONPATH}
```

*macOS:*

```bash
cd drake-build
export PYTHONPATH=${PWD}/install/lib/python3.12/site-packages:${PYTHONPATH}
```

# Building with Bazel

If your organization already uses Bazel for its builds, you can build Drake as
a Bazel external. For sample projects that show how to import Drake as a Bazel
external (either by building Drake from source, or by downloading a pre-compiled
Drake release) please see our gallery of
[drake-external-examples](https://github.com/RobotLocomotion/drake-external-examples),
either
[drake_bazel_external](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_bazel_external)
(to build from source) or
[drake_bazel_download](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_bazel_download)
(to download a precompiled release).

When building Drake from source as Bazel external, we offer flags for
customization. Refer to the comments in
[drake/tools/flags/BUILD.bazel](https://github.com/RobotLocomotion/drake/blob/master/tools/flags/BUILD.bazel)
for details. The `drake_bazel_external` example demonstrates a few of the flags.
If you enable any of proprietary solvers flags, then you must first install
the solver and set environment variables per the
[Proprietary Solvers](/bazel.html#proprietary-solvers) instructions.

There is no way to install Drake from Bazel. To install Drake, use CMake (see
above).

# Making changes to Drake

Drake developers use Bazel for development. Refer to our [Bazel
instructions](/bazel.html) for details.
