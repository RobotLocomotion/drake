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

All of the below commands should be run from the **drake-distro** working
directory, i.e., at the root directory of the git clone.

Build the entire project::

  bazel build //...

Build only the systems (sub)directories::

  bazel build drake/systems/...

Build and test the entire project::

  bazel test //...

Run one specific test::

  bazel test drake/systems/framework/test/diagram_test

Run one specific test in debug mode (including all dependencies also compiled
in debug mode)::

  bazel test -c dbg drake/systems/framework/test/diagram_test

Run a test under memcheck::

  bazel test --config=memcheck drake/systems/framework/test/diagram_test

Run a test under AddressSanitizer::

  bazel test --config=asan drake/systems/framework/test/diagram_test

Build the entire project using clang (on Ubuntu)::

  bazel build --config=clang ...

Run one of the compiled programs manually, from the build outputs directory
(note that ``bazel-bin`` is a directory name here)::

  bazel-bin/drake/systems/framework/test/diagram_test

For more reading about target patterns, see:
https://bazel.build/versions/master/docs/bazel-user-manual.html#target-patterns
