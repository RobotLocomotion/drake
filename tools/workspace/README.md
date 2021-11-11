
This `//tools/workspace/...` package tree contains files related to Drake's
Bazel build system, specifically files relating to downloading and/or compiling
third-party software, as cited by Drake's top-level `/WORKSPACE` file.

# File layout

Files directly in the `//tools/workspace` package are generic helpers,
unrelated to any one third-party library.  Files in sub-packages such as
`//tools/workspace/eigen` are specific to their third-party software; the
sub-package is named to match that software's corresponding name in the
`//tools/workspace/default.bzl` file.

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

# Semi-automated monthly upgrades

Drake maintainers will use the ``bazel-bin/tools/workspace/new_release`` tool
to report and upgrade any out-of-date externals.  The process is as follows:

Begin from an up-to-date checkout of Drake ``master``.

Read the documentation in ``//tools/workspace:new_release``.  In particular,
note that only certain operating systems are supported by that tool.  If you
haven't yet created GitHub API token per those docs, do that now.

Open a new branch:

```
  git checkout -b upgrades
```

Run the "upgrades needed" report.  Copy its output into a temporary text file
so that you can easily refer back to it as you proceed.

```
  bazel build //tools/workspace:new_release
  bazel-bin/tools/workspace/new_release
```

For each external in the report, add a commit that upgrades it, as follows:

Run the script to perform one upgrade (for some external "foo"):

```
  bazel-bin/tools/workspace/new_release --upgrade=foo
```

If the automated update doesn't succeed, then you'll need to make the edits
manually.  Ask for help in drake developers slack channel for ``#build``.

If the automated update succeeded, then inspect the diffs in your editor.
Some diffs will have an instructive comment nearby, e.g., "If you change
this commit, then you need to do X, Y, Z afterward."  Follow any advice
that you find.

Run ``bazel test --config lint //...`` as a sanity check of the edits.

Once all edits are complete, add the commit using the instructions that
were printed by ``new_release``, e.g.:

```
  git add tools/workspace/rules_python/repository.bzl
  git commit -m'[workspace] Upgrade rules_python to latest release A.B.C'
```

Be sure to add any extra files that you edited, too.

Repeat this process for all upgrades.  You can re-run the ``new_release``
report anytime, to get the remaining items that need attention.

Each external being upgraded should have exactly one commit that does the
upgrade, and each commit should only impact exactly one external.  If we
find any problem with an upgrade, we need to be able to revert the commit
for just that one external upgrade, leaving the other upgrades intact.

Once all upgrades are ready, open a Drake pull request and label it
``status: commits are properly curated``.  Open the Reviewable page and
change the drop-down that says "Combine commits for review" to choose
"Review each commit separately" instead.

Once the all Jenkins builds of the pull request have passed,
additionally launch a macOS build, per

https://drake.mit.edu/jenkins.html#scheduling-an-on-demand-build

Once the macOS build passes, assign the pull request for review.

If any of the changes were out of the ordinary (i.e., if they were anything
more than just editing the version numbers and checksums), please be sure
that those changes are explained in the pull request overview text, so that
your reviewer(s) can understand the context and/or reasoning behind them.

# Changing the version of third-party software manually

The instructions for updating third-party software differ depending on how
Drake is obtaining that software.

Most third-party software used by Drake will be incorporated via files named
`//tools/workspace/foo:repository.bzl` where `foo` is the name of the software
(`boost`, `eigen`, `vtk`, etc.).  Consult that file to check which download or
installation helper is used; find the helper in the list below to continue.

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

Mimic an existing example to complete the process, e.g., look at
`//tools/workspace/tinyobjloader` and mimic the `repository.bzl` and
`package.BUILD.bazel` files.
