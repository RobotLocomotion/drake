---
title: Developing Drake using Bazel
---

# New Users

For first-time users, we strongly suggest using one of the pre-compiled binaries
described on our [installation](/installation.html) page. If you need to install
Drake from source please refer to the
[CMake instructions](from_source.html#supported-configurations).
If your organization already uses Bazel for its builds, you can build Drake as a
Bazel external; see [Building with Bazel](/from_source.html#building-with-bazel).

This page describes how Drake Developers (and contributors making pull requests)
should develop and test their changes locally.

# Introduction

Drake's primary build system is [Bazel](https://bazel.build/), which our
developers use to build and test locally and in CI. Bazel enables speedy
development via fine-grained caching of all actions, including tests.

For end users, Drake also offers a CMake build system wrapper that invokes Bazel
as a subprocess and maps CMake build options into Bazel build options, so that
users can install Drake via standard tools without knowing anything about Bazel.
See the
[CMake instructions](from_source.html#supported-configurations)
for details.

# Getting Drake

Run:

```bash
# Get the sources.
git clone --filter=blob:none https://github.com/RobotLocomotion/drake.git

# Install the developer dependencies.
drake/setup/install_prereqs --developer
```

We suggest you keep the default clone directory name (``drake``) and not rename
it (e.g., ``drake2``). The CLion integration will suffer if the checkout is not
named ``drake``. (See [CLion IDE setup](clion.html) for details.)

## Using a fork of Drake

The above ``git clone`` command will configure Drake's primary repository as a
remote called ``origin``. If you plan to fork Drake for development, we
recommend that you configure your fork of Drake's primary repository as the
``origin`` remote and Drake's primary repository as the ``upstream``
remote. This can be done by executing the following commands:

```bash
cd drake
git remote set-url origin git@github.com:[your github user name]/drake.git
git remote add upstream https://github.com/RobotLocomotion/drake.git
git remote set-url --push upstream no_push
```

We recommend that you
[setup SSH access to github.com](https://help.github.com/articles/adding-a-new-ssh-key-to-your-github-account/)
to avoid needing to type your password each time you access it.

# Developing Drake using Bazel

To build or test Drake, run `bazel build` or `bazel test` with the desired
target label (and optional configuration options if desired). We give some
typical examples below; for more reading about target patterns, see
[target patterns](https://bazel.build/docs/user-manual#target-patterns).

On Ubuntu, the default compiler is GCC. On macOS, the default compiler is the
Apple LLVM compiler. To use Clang on Ubuntu, add ``--config=clang`` after any
`bazel` command.

Cheat sheet for operating on the entire project:

```bash
cd /path/to/drake
bazel build //...                 # Build the entire project.
bazel test //...                  # Build and test the entire project.

bazel build --config=clang //...  # Build using Clang on Ubuntu.
bazel test --config=clang //...   # Build and test using Clang on Ubuntu.
```

* The "``//``" means "starting from the root of the project".
* The "``...``" means "everything, recursing into subdirectories".

You may use relative pathnames if your shell's working directory is not at the
project root:

```bash
cd /path/to/drake/common
bazel build ...                   # Build everything in common and its child subdirectories.
bazel test ...                    # Test everything in common and its child subdirectories.
bazel build //...                 # Build the entire project.
bazel test //...                  # Build and test the entire project.
```

* As before, the "``...``" means "everything, recursing into subdirectories".
    * In the first two lines we did not precede "``...``" with "``//``", so the
      search begins in the current directory (``common``) and not from the
      ``drake`` root.
    * In the second two lines we used the "``//``" prefix to specify the project
      root, so we're back to operating on the entire project even though
      ``common`` is still our shell's current working directory.

Cheat sheet for operating on specific portions of the project:

```bash
cd /path/to/drake
bazel build common/...                               # Build everything in common and its child subdirectories.
bazel build common                                   # Build libcommon.
bazel build common:polynomial                        # Build libpolynomial.
bazel build common:all                               # Build everything in common but NOT its children.

bazel test common:polynomial_test                    # Run one test.
bazel test --config=debug common:polynomial_test       # Run one test in debug mode.
bazel test --config=memcheck common:polynomial_test  # Run one test under memcheck (valgrind).
bazel test --config=fastmemcheck common:all          # Run common's tests under memcheck, with minimal recompiling.
bazel test --config=kcov common:polynomial_test      # Run one test under kcov (see instructions below).
bazel build --config=debug common:polynomial_test && \
  gdb bazel-bin/common/polynomial_test               # Run one test under gdb.

bazel test --config=debug --config=clang --config=asan common:polynomial_test  # Run one test under AddressSanitizer.

bazel test --config lint //...                       # Only run style checks; don't build or test anything else.
```

* The "``:``" syntax separates target names from the directory path of the
  ``BUILD`` file they appear in. In this case, for example,
  ``drake/common/BUILD`` specifies
  ``drake_cc_googletest(name = "polynomial_test")``.
* Note that the configuration switches (``-c`` and ``--config``) influence the
  entire command. For example, running a test in ``dbg`` mode means that its
  prerequisite libraries are also compiled and linked in ``dbg`` mode.

On Linux, OpenMP is enabled by default; to disable it, use
`--config=without_openmp`.

## Running with Flags

### Example programs

In general, to figure out what binary-specific arguments are available, add
"``-- --help``" to your ``bazel run`` command. An an example,

```bash
bazel run //examples/acrobot:run_passive -- --help
```

The bare ``--`` separates Bazel arguments from the program's arguments.

## Unit tests

For running tests, you may pass custom arguments to the test program via
[--test_arg](https://docs.bazel.build/versions/main/user-manual.html#flag--test_arg).

For a C++ unittest that uses ``drake_cc_googletest``, for example:

```bash
bazel test //multibody/plant:multibody_plant_test --test_output=streamed --nocache_test_results --test_arg=--gtest_filter='*SimpleModelCreation*'
```

For a Python unittest that uses ``drake_py_unittest``, for example:

```bash
bazel test //bindings/pydrake:py/symbolic_test --test_output=streamed --nocache_test_results --test_arg=--trace=user --test_arg=TestSymbolicVariable
```

# Updating BUILD files

We use the "``buildifier``" tool to auto-format our ``BUILD`` files (in the same
spirit as ``clang-format`` formatting C++ code):

```bash
cd /path/to/drake
bazel-bin/tools/lint/buildifier --all         # Reformat all Bazel files.
bazel-bin/tools/lint/buildifier common/BUILD  # Only reformat one file.
```

If a BUILD file is mis-formatted, you we see a test failure with instructions
how to fix the problem.

In most cases the ``bazel-bin/tools/lint/buildifier`` will already be compiled
by the time you need it.  In case it's absent, you can compile it via:

```bash
cd /path/to/drake
bazel build //tools/lint:buildifier
```

# Proprietary Solvers

The Drake Bazel build currently supports the following proprietary solvers:

* Gurobi 13.0
* MOSEK™ 11.1
* SNOPT 7.4

Important note: Running Gurobi or MOSEK™ requires a license, which will often
have a maximum number of programs allowed to use the solver at once. To avoid
hitting the concurrency limit when running `bazel test //some/package/...` or
`bazel test //...`, use the flag `--local_test_jobs=N` set to the number of
license seats you want to use.

## Gurobi

### Install on Ubuntu

1. Register for an account on [https://www.gurobi.com](https://www.gurobi.com).
2. Set up your Gurobi license file in accordance with Gurobi documentation.
3. ``export GRB_LICENSE_FILE=/path/to/gurobi.lic``.
4. Download ``gurobi13.0.1_linux64.tar.gz``. You may need to manually edit the
   URL to get the correct version.
5. Unzip it. We suggest that you use ``/opt/gurobi1301`` to simplify working
   with Drake installations.
6. If you unzipped into a location other than ``/opt/gurobi1301``, then call
   ``export GUROBI_HOME=GUROBI_UNZIP_PATH/linux64`` to set the path you used,
   where in ``GUROBI_HOME`` folder you can find ``bin`` folder.

Drake supports any patch version of Gurobi 13.0. At time of writing, the most
recent available version was 13.0.1; if using a newer patch version, the paths
and file names above should be adjusted accordingly.

### Install on macOS

1. Register for an account on [http://www.gurobi.com](http://www.gurobi.com).
2. Set up your Gurobi license file in accordance with Gurobi documentation.
3. ``export GRB_LICENSE_FILE=/path/to/gurobi.lic``
4. Download and install ``gurobi13.0.1_macos2_universal.pkg``.

To confirm that your setup was successful, run a test that requires Gurobi:

  ```bazel test --@drake//tools/flags:with_gurobi=True //solver:gurobi_solver_test```

## MOSEK

The Drake Bazel build system downloads MOSEK™ 10.0.18 automatically. No manual
installation is required.  Set the location of your license file as follows:

  ```export MOSEKLM_LICENSE_FILE=/path/to/mosek.lic```

To confirm that your setup was successful, run a test that requires MOSEK™:

  ```bazel test --@drake//tools/flags:with_mosek=True //solvers:mosek_solver_test```

## SNOPT

Drake provides two mechanisms to include the SNOPT sources.  One mechanism is
to provide your own SNOPT source archive.  The other mechanism is via access to
a private RobotLocomotion git repository.

### Using your own source archive

1. Download the SNOPT sources from the distributor in ``.tar.gz`` format (e.g.,
   named ``snopt7.4.tar.gz``).
2. ``export SNOPT_PATH=/home/username/Downloads/snopt7.4.tar.gz``

Using the RobotLocomotion git repository

1. Obtain access to the private RobotLocomotion/snopt GitHub repository.
2. [Set up SSH access to github.com](https://help.github.com/articles/adding-a-new-ssh-key-to-your-github-account/).
3. ``export SNOPT_PATH=git``

Test the build (for either mechanism)

To confirm that your setup was successful, run a test that requires SNOPT:

  ```bazel test --@drake//tools/flags:with_snopt=True //solvers:snopt_solver_test```

SNOPT support has some known problems on certain programs (see drake issue
[#10422](https://github.com/RobotLocomotion/drake/issues/10422) for a summary).

# Other optional dependencies

# Optional Tools

The Drake Bazel build system has integration support for some optional
development tools:

* kcov -- test coverage analysis
* docker -- debug CI failures using a virtual machine

## kcov

``kcov`` can analyze coverage for any binary that contains DWARF format
debugging symbols, and produce nicely formatted browse-able coverage reports.

Drake's ``kcov`` build system integration is only supported on Ubuntu, not
macOS.

In some cases, running kcov builds and regular builds from the same source
tree will lead to Bazel error messages like "this rule is missing dependency
declarations".  To resolve that problem, either run the kcov build from a
fresh checkout, or else run a ``bazel clean``.

To analyze test coverage, run one (or more) tests under ``kcov``:

```
bazel test --config=kcov common:polynomial_test
```

Note that it disables compiler-optimization (``-O0``) to have a better and more
precise coverage report.  If you have trouble with kcov and unoptimized programs,
you can turn it back on by also supplying ``--copt -O2``.

For each test program, individual coverage reports are written to per-target
directories.  Use the ``kcov_tool`` to merge coverage data into a new directory:

```
tools/dynamic_analysis/kcov_tool merge [OUTPUT-DIR]
```

To view the merged data, browse to ``index.html`` in the OUTPUT-DIR.

In a local developer workspace, coverage data may accumulate over successive
build jobs, even if source files or other dependencies have changed. The stale
data would be scattered within the directory tree linked as
``bazel-testlogs``. To clear out old data, use ``kcov_tool clean``:

```
tools/dynamic_analysis/kcov_tool clean
```

### Drake bazel rules and kcov

Some Drake-specific bazel rules (e.g. `drake_cc_googletest`) use various
heuristics to skip certain tests in `kcov` builds. This may hinder developers
trying to use `kcov` locally on specific tests. For example:

```
bazel test --config=kcov //common:temp_directory_test
```

results in:
```
ERROR: No test targets were found, yet testing was requested
```

To force execution with kcov, add an empty `test_tag_filters` option:
```
bazel test --config=kcov --test_tag_filters= //common:temp_directory_test
```

## docker

Drake supports multiple Ubuntu versions. Developers who need to debug
a CI failure on an Ubuntu version other than their desktop version can
use Docker:

Install docker:

```
$ sudo apt install docker.io
$ sudo usermod -aG docker $USER
$ newgrp docker
```

Boot a virtual machine, clone Drake, install prereqs, and develop as usual:

```
$ docker run --rm -it ubuntu:26.04
# apt update
# apt install -y git sudo
# echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
# su - ubuntu
$ git clone --depth=1 https://github.com/RobotLocomotion/drake.git
$ cd drake
$ git fetch origin pull/NNNNN/head && git checkout FETCH_HEAD
$ setup/install_prereqs -y --developer
$ bazel test //common:fmt_test
```

When you exit the virtual machine it will be deleted, so be sure to
capture what you need before exiting.
