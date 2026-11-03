
This directory contains copies of CMake-generated source code that is too
difficult to re-generate from Bazel.

- vtkRenderingOpenGL2ObjectFactory: supports the vtkAutoInit dependency
injection framework as related to VTK rendering. The `*.cxx` file has further
been customized by hand to comment out the overrides for classes that Drake
doesn't compile (see `srcs_glob_exclude` in settings.bzl) and for the render
window (the choice of which is platform-specific).

The `object_factory_lint_test` cross-checks the overrides in Drake's copy
against the VTK source; if it fails, these files need to be refreshed.

TODO(jwnimmer-tri) Add some kind of vtk-upgrade helper script, that refreshes
these files. In the meantime, when upgrading VTK you can refresh these files by
hand:

(1) Get the VTK source (e.g., clone VTK and check out the git sha you want, or
unzip the VTK archive that the bazel workspace refers to).

(2) Run CMake to to generate the upstream flavor of these files:
  $ sudo apt install ninja-build
  $ cd vtk
  $ mkdir build
  $ cd build
  $ cmake .. -Werror=dev -G Ninja -DVTK_MODULE_ENABLE_VTK_RenderingOpenGL2=YES

(3) Use your IDE's diff-merge tool to meld the new copies of the files ...
  vtk/build/Rendering/OpenGL2/vtkRenderingOpenGL2ObjectFactory.h
  vtk/build/Rendering/OpenGL2/vtkRenderingOpenGL2ObjectFactory.cxx
... with the files in Drake's source tree. The Drake customizations that we
should retain are summarized above in this README, and also highlighted with
comments in the C++ code. Other differences versus upstream should probably
be melded back into the Drake copy. Anything that passes `bazel test //...`
on all supported platforms in CI is sufficient.
