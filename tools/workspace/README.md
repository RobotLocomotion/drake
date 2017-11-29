
This `//tools/workspace/...` package tree contains files related to Drake's
Bazel build system, specifically files relating to downloading and/or compiling
third-party software, as cited by Drake's top-level `/WORKSPACE` file.

File layout
-----------

Files directly in the `//tools/workspace` package are generic helpers,
unrelated to any one third-party library.  Files in sub-packages such as
`//tools/workspace/eigen` are specific to their third-party software; the
sub-package is named to match that software's corresponding entry in the
top-level `/WORKSPACE` file.

Files named `BUILD.bazel` denote the package structure within our sub-folders;
in the case of the `//tools/workspace/...` packages, these are largely just
visibility declarations.

Files named `*.BUILD.bazel` are Drake-specific build rules for external
libraries or tools that do not natively support Bazel:
  https://docs.bazel.build/versions/master/external.html#depending-on-non-bazel-projects

Files named `*.bzl` are Skylark extensions, usually for workspace rules:
  https://docs.bazel.build/versions/master/skylark/concepts.html
  https://docs.bazel.build/versions/master/be/workspace.html

Files named `*.cps` are written the in Common Package Specification language,
which provides the necessary information for a project to be consumed by other
projects via `cps2cmake`-generated `*.cmake` files.

Adding new third-party software
-------------------------------

TODO(jwnimmer-tri) Write me.
