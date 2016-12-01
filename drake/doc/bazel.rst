******************
Bazel build system
******************

Drake is experimenting with the Bazel build system, but we do not officially
support Bazel builds.  For more information, see:

 * https://bazel.build/
 * https://github.com/RobotLocomotion/drake/issues/3129

Bazel Installation
==================

Refer to the Bazel installation instructions:
https://bazel.build/versions/master/docs/install.html

We currently develop with Bazel 0.3.1 using Ubuntu Xenial.

Drake clone and platform setup
==============================

The one-time platform setup is the same as with CMake build:

 - Start with a **git clone** of drake, per the :ref:`Getting Drake
   <getting_drake>` instructions.

 - Continue with the *"Mandatory platform specific instructions"* on the same
   page.

   - Note that **only** the software installation steps are required.
   - Do not set extra environment variables (Bazel ignores them).
   - Be sure that **ccache is not on your default $PATH**, e.g.,
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

Run one of the compiled programs manually, from the build outputs directory
(note that ``bazel-bin`` is a directory name here)::

  bazel-bin/drake/systems/framework/test/diagram_test

For more reading about target patterns, see:
https://bazel.build/versions/master/docs/bazel-user-manual.html#target-patterns
