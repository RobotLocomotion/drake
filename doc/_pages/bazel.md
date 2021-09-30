---
title: Bazel build system
---

Drake's primary build system is Bazel.  For more information about Bazel, see
[https://bazel.build/](https://bazel.build/).

Drake also offers a CMake build system wrapper that invokes Bazel under the
hood.

# Bazel Installation

Follow Drake's
[platform-specific setup instructions](/from_source.html#mandatory-platform-specific-instructions)
to install Bazel.

# Drake clone and platform setup

* Start with a **git clone** of drake, per the [Getting Drake](/from_source.html#getting-drake)
  instructions.
* Continue with the *"Mandatory platform specific instructions"* on the same
  page.

# Developing Drake using Bazel

To build or test Drake, run **bazel build** or **bazel test** with the desired
target label (and optional configuration options if desired).  We give some
typical examples below; for more reading about target patterns, see:
[https://docs.bazel.build/versions/master/user-manual.html#target-patterns](https://docs.bazel.build/versions/master/user-manual.html#target-patterns).

On Ubuntu, the default compiler is the first ``gcc`` compiler in the
``PATH``, usually GCC 7.5 on Bionic and GCC 9.3 on Focal. On macOS, the default
compiler is the Apple LLVM compiler. To use Clang 9 on Ubuntu, set the ``CC``
and ``CXX`` environment variables before running **bazel build**, **bazel test**
or any other **bazel** commands.

Cheat sheet for operating on the entire project:

```
cd /path/to/drake
bazel build //...                               # Build the entire project.
bazel test //...                                # Build and test the entire project.

CC=clang-9 CXX=clang++-9 bazel build //...      # Build using Clang 9 on Ubuntu.
CC=clang-9 CXX=clang++-9 bazel test //...       # Build and test using Clang 9 on Ubuntu.
```

* The "``//``" means "starting from the root of the project".
* The "``...``" means "everything including the subdirectories' ``BUILD`` files".
    * Contrast with, e.g., the "``bazel build common:*``" explained below, where
      only targets declared *directly* in ``drake/common/BUILD`` are compiled,
      and not the targets in ``drake/common/trajectories/BUILD``.  The "``*``"
      matches targets in that directory; the "``...``" also matches down into
      subdirectories.

You may use relative pathnames if your shell's working directory is not at the
project root:

```
cd /path/to/drake/common
bazel build ...                   # Build everything in common and its child subdirectories.
bazel test ...                    # Test everything in common and its child subdirectories.
bazel build //...                 # Build the entire project.
bazel test //...                  # Build and test the entire project.
```

* As before, the "``...``" above means "everything including subdirectories".
    * In the first two lines we did not precede "``...``" with "``//``", so the
      search begins in the current directory (``common``) and not from the
      ``drake`` root.
    * In the second two lines we used the "``//``" prefix to specify the project
      root, so we're back to operating on the entire project even though
      ``common`` is still our shell's current working directory.

Cheat sheet for operating on specific portions of the project:

```
cd /path/to/drake
bazel build common/...                               # Build everything in common and its child subdirectories.
bazel build common                                   # Build libcommon.
bazel build common:polynomial                        # Build libpolynomial.
bazel build common:*                                 # Build everything in common but NOT its children.

bazel test common:polynomial_test                    # Run one test.
bazel test -c dbg common:polynomial_test             # Run one test in debug mode.
bazel test --config=memcheck common:polynomial_test  # Run one test under memcheck (valgrind).
bazel test --config=fastmemcheck common:*            # Run common's tests under memcheck, with minimal recompiling.
bazel test --config=asan common:polynomial_test      # Run one test under AddressSanitizer.
bazel test --config=kcov common:polynomial_test      # Run one test under kcov (see instructions below).
bazel build -c dbg common:polynomial_test && \
  gdb bazel-bin/common/polynomial_test               # Run one test under gdb.

bazel test --config lint //...                       # Only run style checks; don't build or test anything else.
```

* The "``:``" syntax separates target names from the directory path of the
  ``BUILD`` file they appear in. In this case, for example,
  ``drake/common/BUILD`` specifies ``cc_test(name = "polynomial_test")``.
* Note that the configuration switches (``-c`` and ``--config``) influence the
  entire command. For example, running a test in ``dbg`` mode means that its
  prerequisite libraries are also compiled and linked in ``dbg`` mode.
* For the definitions of the "``--config``" options see ``drake/tools/bazel.rc``.

## Running with Flags

In general, to figure out what binary-specific arguments are available, add
"``-- --help``" to your ``bazel run`` command. If the binary can only run via
``bazel test``, look at [--test_arg](https://docs.bazel.build/versions/master/user-manual.html#flag--test_arg).

If a C++ unittest uses ``gtest`` (e.g. using ``drake_cc_googletest``),
you can specify gtest-specific flags. As an example:

```
bazel run multibody/plant:multibody_plant_test -- --gtest_filter='*SimpleModelCreation*'
```

If a Python unittest is run via ``drake_py_unittest_main.py`` (e.g. using
``drake_py_unittest``), you can specify flags such as ``--trace`` or
``--deprecation_action``. As an example:

```
bazel run bindings/pydrake:py/symbolic_test -- --trace=user --deprecation_action=error
```

## Debugging and profiling on macOS

On macOS, DWARF debug symbols are emitted to a ``.dSYM`` file.  The Bazel
``cc_binary`` and ``cc_test`` rules do not natively generate or expose this
file, so we have implemented a workaround in Drake, ``--config=apple_debug``.
This config turns off sandboxing, which allows a ``genrule`` to access the
``.o`` files and process them into a ``.dSYM``.  Use as follows:

```
bazel build --config=apple_debug path/to/my:binary_or_test_dsym
lldb ./bazel-bin/path/to/my/binary_or_test
```

Profiling on macOS can be done by building with the debug symbols and then running
```
xcrun xctrace record -t "Time Profiler" --launch ./bazel-bin/path/to/my/binary_or_test
```
This will generate a `.trace` file that can be opened in the Instruments app:
```
open -a Instruments myfile.trace
```

For more information, see [https://github.com/bazelbuild/bazel/issues/2537](https://github.com/bazelbuild/bazel/issues/2537).

# Updating BUILD files

Please use the "``buildifier``" tool to format edits to ``BUILD`` files (in the
same spirit as ``clang-format`` formatting C++ code):

```
cd /path/to/drake
bazel-bin/tools/lint/buildifier --all         # Reformat all Bazel files.
bazel-bin/tools/lint/buildifier common/BUILD  # Only reformat one file.
```

In most cases the ``bazel-bin/tools/lint/buildifier`` will already be compiled
by the time you need it.  In case it's absent, you can compile it via:

```
cd /path/to/drake
bazel build //tools/lint:buildifier
```

# Proprietary Solvers

The Drake Bazel build currently supports the following proprietary solvers:

* Gurobi 9.0.2
* MOSEK 9.2
* SNOPT 7.4

## Gurobi 9.0.2

### Install on Ubuntu

1. Register for an account on [https://www.gurobi.com](https://www.gurobi.com).
2. Set up your Gurobi license file in accordance with Gurobi documentation.
3. ``export GRB_LICENSE_FILE=/path/to/gurobi.lic``.
4. Download ``gurobi9.0.2_linux64.tar.gz``
5. Unzip it.  We suggest that you use ``/opt/gurobi902`` to simplify working with Drake installations.
6. If you unzipped into a location other than ``/opt/gurobi902``, then call ``export GUROBI_HOME=GUROBI_UNZIP_PATH/linux64`` to set the path you used, where in ``GUROBI_HOME`` folder you can find ``bin`` folder.

### Install on macOS

1. Register for an account on [http://www.gurobi.com](http://www.gurobi.com).
2. Set up your Gurobi license file in accordance with Gurobi documentation.
3. ``export GRB_LICENSE_FILE=/path/to/gurobi.lic``
4. Download and install ``gurobi9.0.2_mac64.pkg``.

To confirm that your setup was successful, run the tests that require Gurobi:

  ```bazel test --config gurobi --test_tag_filters=gurobi //...```

The default value of ``--test_tag_filters`` in Drake's ``bazel.rc`` excludes
these tests.  If you will be developing with Gurobi regularly, you may wish
to specify a more convenient ``--test_tag_filters`` in a local ``.bazelrc``.
See [https://docs.bazel.build/versions/master/user-manual.html#bazelrc](https://docs.bazel.build/versions/master/user-manual.html#bazelrc).

## MOSEK

The Drake Bazel build system downloads MOSEK 9.2.33 automatically.  No manual
installation is required.  Set the location of your license file as follows:

  ```export MOSEKLM_LICENSE_FILE=/path/to/mosek.lic```

To confirm that your setup was successful, run the tests that require MOSEK:

  ```bazel test --config mosek --test_tag_filters=mosek //...```

The default value of ``--test_tag_filters`` in Drake's ``bazel.rc`` excludes
these tests.  If you will be developing with MOSEK regularly, you may wish
to specify a more convenient ``--test_tag_filters`` in a local ``.bazelrc``.
See [https://docs.bazel.build/versions/master/user-manual.html#bazelrc](https://docs.bazel.build/versions/master/user-manual.html#bazelrc).

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

To confirm that your setup was successful, run the tests that require SNOPT:

  ```bazel test --config snopt --test_tag_filters=snopt //...```

The default value of ``--test_tag_filters`` in Drake's ``bazel.rc`` excludes
these tests.  If you will be developing with SNOPT regularly, you may wish
to specify a more convenient ``--test_tag_filters`` in a local ``.bazelrc``.
See [https://docs.bazel.build/versions/master/user-manual.html#bazelrc](https://docs.bazel.build/versions/master/user-manual.html#bazelrc).

SNOPT support has some known problems on certain programs (see drake issue
[#10422](https://github.com/RobotLocomotion/drake/issues/10422) for a summary).

# Optional Tools

The Drake Bazel build system has integration support for some optional
development tools:

* kcov -- test coverage analysis

## kcov

``kcov`` can analyze coverage for any binary that contains DWARF format
debugging symbols, and produce nicely formatted browse-able coverage reports.

Drake's ``kcov`` build system integration is only supported on Ubuntu, not
macOS.

To use kcov on Ubuntu 18.04 (Bionic), you must first run Drake's
``install_prereqs`` setup script using the ``--with-kcov`` option. On Ubuntu
20.04 (Focal), the option is ignored. The macOS ``install_prereqs`` setup
script does not install kcov, and passing a ``--with-kcov`` option is an error.

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
