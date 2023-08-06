
This directory contains copies of CMake-generated source code that is too
difficult to re-generate from Bazel.

- vtkRenderingOpenGL2ObjectFactory: supports the vtkAutoInit dependency
injection framework as related to VTK rendering.

TODO(jwnimmer-tri) Add some kind of vtk-upgrade helper script, that refreshes
these files and/or cross-checks the result.
