
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

Files named `repository.bzl` are repository rules, and intended to be a stable
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
- Create `tools/workspace/foo/repository.bzl` that declares a
  `foo_repository()` macro or rule.
- In Drake's top-level `WORKSPACE`, `load()` the new `repository.bzl` file and
  call `foo_repository(name = "foo")`.
- TODO(jwnimmer-tri) Write the rest of this.

Changing the version of third-party software
--------------------------------------------

To temporarily use a local copy of a `github_archive`, within the relevant
`//tools/workspace/foo:repository.bzl` file add a `local_repository_archive`
argument to its `github_archive` macro call pointing at a local checkout, e.g.:

    github_archive(
        name = "foobar",
        local_repository_override = "/path/to/local/foo/bar",
        repository = "foo/bar",
        commit = "0123456789abcdef0123456789abcdef01234567",
        sha256 = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",  # noqa
    )

This allows for easy editing and debugging (e.g., adding logging) temporarily.
Removing the `local_repository_override` reverts to using the given `commit`
and ignores the local checkout.

To use a new upstream revision, change the `commit` argument to refer to a
different revision, then comment out the `sha256` argument and run `bazel
build`.  Bazel's fetch step will download the new version and then complain
about a checksum mismatch.  Paste the new checksum into the `sha256` argument
and remove its commenting-out.  Then, `bazel build` should succeed.
