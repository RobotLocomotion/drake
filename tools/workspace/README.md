
This `//tools/workspace/...` package tree contains files related to Drake's
Bazel build system, specifically files relating to downloading and/or compiling
third-party software, used by Drake's top-level `/MODULE.bazel` file.

# Overview of external dependencies

Drake's primary build system is Bazel, so we'll first explain how Drake's
externals are expressed to Bazel, and then in the next sub-section explain
the CMake interface wrapped around Bazel.

## Bazel modules and module extensions

Drake declares its dependencies using Bazel Modules and Module Extensions:
- https://bazel.build/external/overview
- https://bazel.build/external/extension

When the Bazel Central Registry (https://registry.bazel.build/) contains the
module we need, we prefer to use that mechanism. Otherwise, we write our own
module extension to incorporate the external. Either way, our `MODULE.bazel`
file lists every external that we directly use; refer to its comments for more
details.

Drake also offers configuration flags to govern the externals, e.g., to:
- select where (groups of) externals come from, or
- opt-in to (or opt-out) of a specific external.
Refer to the documentation in `tools/flags/BUILD.bazel` for details. Note that
the `BUILD.bazel` file specifies default values, but when Drake is the main
module (not a dependency), Drake changes some defaults using bazelrc files.

When an external has flag(s) that govern where it comes from, that detail is
encapsulated under an alias. For example, where Eigen comes from is governed by
flags, but the following text in `MODULE.bazel` (either in Drake, or any
downstream project that uses Drake) is the same no matter the flag.

```
drake_dep_repositories = use_extension(
    "@drake//tools/workspace:default.bzl",
    "drake_dep_repositories",
)
use_repo(drake_dep_repositories, "eigen")
```

## CMake interface

For projects that use Drake as an external but prefer CMake, Drake also provides
a `CMakeLists.txt` wrapper that configures and installs Drake. The wrapper
offers not only standard options (e.g., `CMAKE_BUILD_TYPE`) but also options
unique to Drake. Refer to https://drake.mit.edu/from_source.html for details.

On the front end, the CMake wrapper converts CMake options to Bazel flags by
generating custom bazelrc, MODULE, and BUILD files using the templates in
`drake/cmake/...`, as well as build command(s) that shell out to Bazel to
run the `//:install` command.

The `//:install` command uses Drake's `install.bzl` logic to gather and copy
files to their destination. One key part is the `//tools/install/libdrake` rules
to generate and install a `drake-config.cmake` script. Downstream projects will
use that script to find details about the installed copy of Drake, including the
version number, include paths and library paths, additional dependencies, and
available libraries.

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
libraries or tools that do not natively support Bazel.

Files named `repository.bzl` are repository rules, and (when their name doesn't
contain "internal") are intended to be a stable entry point for other Bazel
projects to refer to the same dependencies that Drake is using:
  https://bazel.build/extending/repo

Per the [Stability Guidelines](https://drake.mit.edu/stable.html), externals
named as "internal" or otherwise documented to be "internal use only" are
not subject to any deprecation guarantees.

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
  bazel run //tools/workspace:new_release
```

For each external in the report, add a commit that upgrades it.  Typically,
this can be done by running the script to perform one upgrade (for some
external "foo"):

```
  bazel run //tools/workspace:new_release -- --lint --commit foo
```

If the automated update doesn't succeed and the error message is
"Reversed (or previously applied) patch detected!" and refers to a patch
named /upstream/, try to remove the patch from Drake by deleting the patch
file and removing its mention in the repository.bzl. If the update then works,
the deletions can be amended into the commit for that external.

If it still fails, or for any other errors, you'll need to make the edits
manually. Ask for help in drake developers slack channel for ``#build``.

If the automated update succeeded, check the output of ``new_release`` for any
additional steps that need to be manually performed to complete the upgrade.
Follow any advice that is given.

If you didn't use ``--lint`` earlier, or need to re-test, run
``bazel test --config lint //...`` as a sanity check of the changes.

If any edits are needed, stage the changes and amend the commit using
``git commit --amend``.

Repeat this process for all upgrades.  You can re-run the ``new_release``
report anytime, to get the remaining items that need attention.  You can also
list several externals to try to update at once, although this will complicate
making changes to those commits if needed.  Note that some externals are
reported as "may need upgrade".  This means that ``new_release`` is not able
to automatically determine whether an upgrade is needed; therefore, these
should always be upgraded.  (If no upgrade is needed, the upgrade will do
nothing and will not create a commit.)

Each external being upgraded should have exactly one commit that does the
upgrade, and each commit should either a) only impact exactly one external, or
b) impact exactly those externals of a cohort which need to be upgraded.  If we
find any problem with an upgrade, we need to be able to revert the commit
for just that one external upgrade, leaving the other upgrades intact.
The ``new_release`` will automatically upgrade all externals of a cohort in a
single operation.

Once all upgrades are ready, open a Drake pull request and label it
``status: commits are properly curated``.  Open the Reviewable page and
change the drop-down that says "Combine commits for review" to choose
"Review each commit separately" instead.

If any packages suggested that additional pre-merge testing is needed,
launch jobs as specified in the output of ``new_release``.

Once all of the Jenkins builds of the pull request have passed, assign the
pull request for review. If the pull request contains no especially complicated
changes, it may be assigned to the on-call platform reviewer and labelled
``status: single reviewer ok``.

For any non-trivial changes (i.e., changes that go beyond changing version
numbers, checksums, or trivial fixups to patch files or code spelling), do not
attempt to fix the problems just because you are accountable for the routine
upgrade procedure every month. As a rule of thumb, if you need to spend more
than 5-10 minutes on an upgrade, you should defer the work to a separate pull
request:

* open a pull request with the WIP patch for that one specific external;

* ensure that the Jenkins output shows the problem (e.g., trigger any extra
  non-default builds that failed);

* assign it to the feature owner associated with that external (to find out who
  that is, ask for help in the drake developers ``#build`` slack channel); and

* omit it from the monthly upgrade pull request.

The main objective of the monthly upgrade is to ensure that we stay on top of
problematic changes from upstream. If we discover such problems, we want to
bring them to the attention of the feature owner; their steering should provide
the most efficient path to resolve the problem.

If an external required non-trivial changes, even if you were able to make the
changes yourself, consider separating that external into its own pull request
and assigning it to the associated feature owner.

## drake-external-examples

The
[drake_cmake_external](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_cmake_external)
example demonstrates the use of some of Drake's CMake options for user-provided
external packages as part of the public API. While users may be able to provide
their own versions which don't necessarily match those officially supported by
Drake, the official examples should match and be kept up-to-date as part of the
monthly upgrades process.

The packages with their canonical versions are listed in Drake's `MODULE.bazel`,
and the corresponding user-provided versions in drake-external-examples are
found in the calls to `ExternalProject_Add` in the example's
[CMakeLists.txt](https://github.com/RobotLocomotion/drake-external-examples/blob/main/drake_cmake_external/CMakeLists.txt).
Any time these packages are upgraded in Drake (via the steps above, or separate
from the monthly upgrades), use the
[automated upgrade script](https://github.com/RobotLocomotion/drake-external-examples/blob/main/private/upgrade_cmake_externals.py)
to create a pull request on drake-external-examples. See the script for its
usage requirements; it mirrors the workflow of the `new_release` tool. For the
automated upgrades, it's best to wait to perform upgrades in
drake-external-examples until after the Renovate bot is scheduled to open pull
requests with upgrades, and those upgrades have been merged by Drake
maintainers. See `.github/renovate.json` for the most up-to-date schedule for
automated upgrades.

The continuous integration (via GitHub Actions) in that repository runs the
upgrade script to verify that packages are in sync with Drake, which should
serve as a reminder to maintainers to perform regular upgrades.

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

### Using patch files

When we need to adjust an upstream source release for our purposes, we do that
with `*.patch` files in Drake, not by forking the repository and pointing to
the fork.

Patches should live at tools/workspace/foobar/patches/quux.patch. In case the
change should be upstreamed (even if we haven't filed that pull request yet),
put the patch in tools/workspace/foobar/patches/upstream/quux.patch to make
it easier to search for patches that require follow-up action. In case the
patch should not be upstreamed, you should probably explain why in a comment.

If a change should be upstreamed but our current patch isn't yet groomed to be
good enough, still place it into `.../patches/upstream/quux.patch`; the idea is
to separate patches where some further work is required (whether that be opening
a pull request, or grooming the patch, or even just opening an issue for
discussion) from patches that are quiescent.

## Updating pkg_config_repository software versions

Most `pkg_config_repository` calls refer to libraries provided by the host
operating system (Ubuntu, macOS, etc.).  In most cases, we are stuck with the
host version in order to remain compatible with the wider software ecosystem.
If the host version is problematic, contact the Drake developers for advice.

# Adding new third-party software

The best guide for incorporating new third-party software is to mimic what
Drake does for other third-party software it already uses.  There are roughly
three general approaches, in order of preference:

- Use a program (not library) from the host operating system (e.g., a compiler).
- Download a library or tool as source code and compile it;
- Download a library or tool as binaries.

For libraries, compiling from source is preferred. Typically the library should
be named "internal" and build with hidden and/or namespace-mangled symbols.

Downloading binaries is a last resort, because they are difficult to patch and
difficult to support on multiple platforms.

## Common steps to add new-third-party software

TODO(jwnimmer-tri) Add documentation here about how to validate that the new
software's license is acceptable to use within Drake.

When adding a new external, decide whether it will be covered by our
[Stability Guidelines](https://drake.mit.edu/stable.html). Broadly speaking,
dependencies that come from the host operating system can be covered as
stable, but dependencies that we compile from source code should be internal.
If the new dependency should be in internal, name it like "foo_internal" (not
just "foo") throughout all of the below.

When the software is available in the
[Bazel Central Registry](https://registry.bazel.build/), we generally prefer to
declare it as a `bazel_dep` in our `MODULE.bazel` file, even if we sometimes
conditionally replace it with a Drake-specific build recipe. We don't yet have
good documentation for how to add a `bazel_dep`, but hopefully the existing
dependencies can serve as a starting point.

Otherwise (for module extensions, not available in the Registry), for adopting
new third-party software named "foo", the steps to incorporate it into Drake are
roughly:

- Create a new sub-directory `tools/workspace/foo`.
- Create `tools/workspace/foo/BUILD.bazel` that calls `add_lint_tests()`.
- Create `tools/workspace/foo/repository.bzl` that declares a
  `foo_repository()` macro or rule.  The details are given below.
- Edit `tools/workspace/default.bzl` to load and conditionally call the new
  `foo_repository()` macro or rule.
- Load the module extension in `MODULE.bazel`.
- Add a courtesy mention of the software in `doc/_pages/credits.md`.
- List the `drake_repository_metadata.json` in `tools/workspace/BUILD.bazel`.

When indicating licenses in the source, use the identifier from the
[SPDX License List](https://spdx.org/licenses/).

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
`//tools/workspace/tinyobjloader_internal` and mimic the `repository.bzl` and
`package.BUILD.bazel` files.
