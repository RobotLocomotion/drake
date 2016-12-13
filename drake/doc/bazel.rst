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
target label.

Unless otherwise noted,
all of the below commands should be run from the **drake-distro** working
directory, i.e., at the root directory of the git clone.

Build the entire project::

  bazel build //...

The "``//``" above means "starting from the root of the project";
the "``...``" above means "everything including subdirectories' ``BUILD`` files".

Build only the systems (sub)directories::

  bazel build drake/systems/...

Build and test the entire project::

  bazel test //...

Run one specific test::

  bazel test drake/systems/framework:diagram_test

The "``:``" above is syntax that separates target names from the directory path
of the ``BUILD`` file they appear in.  In this case, for example,
``drake/systems/framework/BUILD`` specifies ``cc_test(name = "diagram_test")``.

Run one specific test in debug mode (including all dependencies also compiled
in debug mode)::

  bazel test -c dbg drake/systems/framework:diagram_test

Run a test under memcheck::

  bazel test --config=memcheck drake/systems/framework:diagram_test

Run a test under AddressSanitizer::

  bazel test --config=asan drake/systems/framework:diagram_test

Build the entire project using clang (on Ubuntu)::

  bazel build --config=clang //...

Run one of the compiled programs manually, from the build outputs directory
(note that ``bazel-bin`` is a directory name here)::

  bazel-bin/drake/systems/framework/test/diagram_test

You can also use relative pathnames if your shell's working directory is not at
the project root.  Run a test using a relative path::

  cd drake/common
  bazel build ...          # Build everything in common/ and its children.
  bazel build :*           # Build everything in common/ but NOT its children.
  bazel test ...           # Test everything in common/ and its children.
  bazel test :*            # Test everything in common/ but NOT its children.

As before, the "``...``" above means "everything including subdirectories".
However, since we did not precede it with "``//``", the search begins in the
current directory (``common``) not from the ``drake-distro`` root.  As before,
the "``:``" above is syntax that separates target names from the directory path
of the ``BUILD`` file they appear in.  In this case there is nothing before the
"``:``" so the context is "in the current directory" (``common``).  The "``*``"
after the colon indicates "all targets" but without searching in
subdirectories.

For more reading about target patterns, see:
https://bazel.build/versions/master/docs/bazel-user-manual.html#target-patterns
