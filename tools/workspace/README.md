
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

Files named `package.BUILD.bazel` are Drake-specific build rules for external
libraries or tools that do not natively support Bazel:
  https://docs.bazel.build/versions/master/external.html#depending-on-non-bazel-projects

Files named `package.bzl` are repository rules, and intended to be a stable
entry point for other Bazel projects to refer to the same dependencies that
Drake is using:
  https://docs.bazel.build/versions/master/skylark/concepts.html
  https://docs.bazel.build/versions/master/be/workspace.html
  https://docs.bazel.build/versions/master/skylark/repository_rules.html

Files named `package.cps` are written the in Common Package Specification
language, which provides the necessary information for a project to be consumed
by other projects via `cps2cmake`-generated `*.cmake` files:
  https://mwoehlke.github.io/cps/
  https://github.com/mwoehlke/pycps

Files names `package-create-cps.py` are code that generates the `package.cps`
file, in cases where its contents should not be hard-coded.

Adding new third-party software
-------------------------------

Referring to some new third-party software as "foo", the steps are roughly:

- Create a new sub-directory `tools/workspace/foo`.
- Create `tools/workspace/foo/BUILD.bazel` that calls `add_lint_tests()`.
- Create `tools/workspace/foo/package.bzl` that declares a `foo_repository()`
  macro or rule.
- In Drake's top-level `WORKSPACE`, `load()` the new `package.bzl` file and
  call `foo_repository(name = "foo")`.
- TODO(jwnimmer-tri) Write the rest of this.
