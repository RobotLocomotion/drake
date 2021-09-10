
This `//tools/workspace/...` package tree contains files related to Drake's
Bazel build system, specifically files relating to downloading and/or compiling
third-party software, as cited by Drake's top-level `/WORKSPACE` file.

# File layout

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

# Changing the version of third-party software

The instructions for updating third-party software differ depending on how
Drake is obtaining that software.

Most third-party software used by Drake will be incorporated via files named
`//tools/workspace/foo:repository.bzl` where `foo` is the name of the software
(`boost`, `eigen`, `vtk`, etc.).  Consult that file to check which download or
installation helper is used; find the helper in the list below to continue.

Drake maintainers can use the bazel-bin/tools/workspace/new_release tool to
report any out-of-date externals.

## Updating github_archive software versions

For software downloaded from github.com and compiled from source, there are two
choices, depending on whether the purpose is exploration from a local clone vs
pushing to Drake master.

### Exploring github_archive changes from a local clone

This allows for easy editing and debugging (e.g., adding logging) temporarily.

To use a local clone of a `github_archive`, first clone the software using
`git` commands manually.  Then, within the relevant
`//tools/workspace/foo:repository.bzl` file add a `local_repository_archive`
argument to the `github_archive` macro call pointing at a local checkout, e.g.:

    github_archive(
        name = "foobar",
        local_repository_override = "/path/to/local/foo/bar",
        repository = "foo/bar",
        commit = "0123456789abcdef0123456789abcdef01234567",
        sha256 = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",  # noqa
    )

Now, `bazel build` and `test` etc. will use the local clone.  When
`local_repository_override` is present, the `repository` and `commit` and
`sha256` arguments are ignored.  Removing the `local_repository_override`
reverts to using the given `commit` and ignores the local checkout.

### Finalizing github_archive changes

To lock in a new upstream revision, edit the `github_archive` macro call within
the relevant `//tools/workspace/foo:repository.bzl` file:

- remove the `local_repository_override` (if it exists),
- change the `commit` argument to refer to a different revision,
- comment out the `sha256` argument, and then
- run `bazel build`.

Bazel's fetch step will attempt to download the new version but then complain
about a checksum mismatch.  Paste the new checksum into the `sha256` argument
and remove its commenting-out.  Then, `bazel build` should succeed.

Briefly check that `//tools/workspace/foo:package.BUILD.bazel` still seems
appropriate; for example, if there are hard-coded version numbers that need to
match the `commit=` tag, they should be updated (this is rare).

Commit and pull-request the changed lines to Drake as usual.  Many changes like
this will be susceptible to Ubuntu vs macOS differences, so please opt-in to
the macOS build(s) in Jenkins before merging, using the instructions at
https://drake.mit.edu/jenkins.html#running-an-on-demand-build.

## Updating pypi_archive software versions

To lock in a new version, change the `version` argument of the `pypi_archive`
call, comment out the `sha256` argument, and then run `bazel build`.  Bazel's
fetch step will attempt to download the new version but then complain about a
checksum mismatch.  Paste the new checksum into the `sha256` argument and
remove its commenting-out.  Then, `bazel build` should succeed.

Commit and pull-request the changed lines to Drake as usual.  Many changes like
this will be susceptible to Ubuntu vs macOS differences, so please opt-in to
the macOS build(s) in Jenkins before merging, using the instructions at
https://drake.mit.edu/jenkins.html#running-an-on-demand-build.

## Updating pkg_config_repository software versions

Most `pkg_config_repository` calls refer to libraries provided by the host
operating system (Ubuntu, macOS, etc.).  In most cases, we are stuck with the
host version in order to remain compatible with the wider software ecosystem.
If the host version is problematic, contact the Drake developers for advice.

# Adding new third-party software

The best guide for incorporating new third-party software is to mimic what
Drake does for other third-party software it already uses.  There are roughly
three general approaches, in order of preference:

- Use a library or tool from the host operating system;
- Download a library or tool as source code and compile it;
- Download a library or tool as binaries.

When the host operating system (macOS, Ubuntu) offers a version of the
software, its best to use that version in order to remain compatible with the
wider software ecosystem.  If the host version is problematic, contact the
Drake developers for advice.

When the host doesn't offer the software, compiling from source is preferred.
Downloading binaries is a last resort, because they are difficult to patch and
difficult to support on multiple platforms.

## Common steps to add new-third-party software

TODO(jwnimmer-tri) Add documentation here about how to validate that the new
software's license is acceptable to use within Drake.

Referring to some new third-party software as "foo", the steps to incorporate
it into Drake are roughly:

- Create a new sub-directory `tools/workspace/foo`.
- Create `tools/workspace/foo/BUILD.bazel` that calls `add_lint_tests()`.
- Create `tools/workspace/foo/repository.bzl` that declares a
  `foo_repository()` macro or rule.  The details are given below.
- Edit `tools/workspace/default.bzl` to load and conditionally call the new
  `foo_repository()` macro or rule.

When indicating licenses in the source, use the identifier from the
[SPDX License List](https://spdx.org/licenses/).

## When using a library from the host operating system

See `glib` for an example.

Update the package setup lists to mention the new package:

- `setup/ubuntu/binary_distribution/packages-bionic.txt` and
  `setup/ubuntu/binary_distribution/packages-focal.txt` with the `libfoo0`
  runtime library;
- `setup/ubuntu/source_distribution/packages-bionic.txt` and
  `setup/ubuntu/source_distribution/packages-focal.txt` with the `libfoo-dev`
  library;
- `setup/mac/binary_distribution/Brewfile` if used in Drake's installed copy;
- `setup/mac/source_distribution/Brewfile` if only used during development (not
  install).

In `tools/workspace/foo/repository.bzl`, use `pkg_config_repository` to locate
a library from the host.

## When downloading a library or tool as source code

For choosing the version or commit to use in `repository.bzl`:

* When upstream provides numbered releases, pin Drake to use the most recent
stable release. Drake maintainers will automatically upgrade to a more recent
stable release on a monthly basis.
* Otherwise, pin Drake to use the most recent commit of the upstream mainline
branch. Drake maintainers will automatically upgrade to a more recent mainline
commit on a monthly basis.
* If the pin policy is unsatisfactory for the case of some specific external,
consult Drake's build system maintainers for advice.

For Git, the mainline branch is typically `master`, whereas for Mercurial it is
`default`.

TODO(jwnimmer-tri) Write the remainder of this section.
