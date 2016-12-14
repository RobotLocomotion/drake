******************
Bazel build system
******************

The Bazel build system is officially supported for a subset of Drake on
Ubuntu Xenial, and is being tested on Ubuntu Trusty and OS X.
For more information, see:

 * https://bazel.build/
 * https://github.com/RobotLocomotion/drake/issues/3129

Bazel Installation
==================

The Ubuntu Xenial platform setup process installs Bazel for you. On other
platforms, refer to the Bazel installation instructions. We use Bazel 0.4.2.
https://bazel.build/versions/master/docs/install.html

Drake clone and platform setup
==============================

The one-time platform setup is the same as for a CMake build:

 - Start with a **git clone** of drake, per the :ref:`Getting Drake
   <getting_drake>` instructions.

 - Continue with the *"Mandatory platform specific instructions"* on the same
   page.

When using Bazel, be sure that **ccache is not on your default $PATH**, e.g.,
``env | grep ccache`` is empty.

Developing Drake using Bazel
============================

To build or test Drake, run **bazel build** or **bazel test** with the desired
target label (and optional configuration options if desired).  We give some
typical examples below; for more reading about target patterns, see:
https://bazel.build/versions/master/docs/bazel-user-manual.html#target-patterns.

Cheat sheet for operating on the entire project::

  cd /path/to/drake-distro
  bazel build //...                 # Build the entire project.
  bazel test //...                  # Build and test the entire project.
  bazel build --config=clang //...  # Build the entire project using clang (on Ubuntu).

- The "``//``" means "starting from the root of the project".
- The "``...``" means "everything including the subdirectories' ``BUILD`` files".

  - Contrast with, e.g., the "``bazel build common:*``" explained below, where
    only targets declared *directly* in ``drake/common/BUILD`` are compiled,
    and not the targets in ``drake/common/trajectories/BUILD``.  The "``*``"
    matches targets in that directory; the "``...``" also matches down into
    subdirectories.

You may use relative pathnames if your shell's working directory is not at the
project root::

  cd /path/to/drake-distro/drake/common
  bazel build ...                   # Build everything in common and its child subdirectories.
  bazel test ...                    # Test everything in common and its child subdirectories.
  bazel build //...                 # Build the entire project.
  bazel test //...                  # Build and test the entire project.

- As before, the "``...``" above means "everything including subdirectories".

  - In the first two lines we did not precede "``...``" with "``//``", so the
    search begins in the current directory (``common``) and not from the
    ``drake-distro`` root.
  - In the second two lines we used the "``//``" prefix to specify the project
    root, so we're back to operating on the entire project even though
    ``common`` is still our shell's current working directory.

Cheat sheet for operating on specific portions of the project::

  cd /path/to/drake-distro/drake
  bazel build common/...                               # Build everything in common and its child subdirectories.
  bazel build common                                   # Build libcommon.
  bazel build common:polynomial                        # Build libpolynomial.
  bazel build common:*                                 # Build everything in common but NOT its children.

  bazel test common:polynomial_test                    # Run one test.
  bazel test -c dbg common:polynomial_test             # Run one test in debug mode.
  bazel test --config=memcheck common:polynomial_test  # Run one test under memcheck (valgrind).
  bazel test --config=asan common:polynomial_test      # Run one test under AddressSanitizer.
  bazel build -c dbg common:polynomial_test && \
    gdb ../bazel-bin/drake/common/polynomial_test      # Run one test under gdb.

- The "``:``" syntax separates target names from the directory path of the
  ``BUILD`` file they appear in.  In this case, for example,
  ``drake/commmon/BUILD`` specifies ``cc_test(name = "polynomial_test")``.
- Note that the configuration switches (``-c`` and ``--config``) influence the
  entire command.  For example, running a test in ``dbg`` mode means that its
  prerequisite libraries are also compiled and linked in ``dbg`` mode.

Updating BUILD files
====================

Please use the "``buildifier``" tool to format edits to ``BUILD`` files (in the
same spirit as ``clang-format`` formatting C++ code)::

  cd /path/to/drake-distro
  tools/buildifier.sh                     # By default, reformats all BUILD files.
  tools/buildifier.sh drake/common/BUILD  # Only reformat one file.
