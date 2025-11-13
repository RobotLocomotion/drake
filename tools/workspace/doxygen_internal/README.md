# Doxygen

Drake pre-builds its Doxygen binary in a Docker container using CMake. Both the
source archive used to build the binary, and the binary itself, are published
at https://drake-mirror.csail.mit.edu; see `//tools/workspace/mirrors.bzl` for
the specific path. The documentation generation then simply downloads the
pre-built binary, rather than downloading and re-building the sources.

This has the advantages of:

* not having to maintain Bazel build rules for Doxygen, and
* providing faster build times.

However, it also introduces additional steps in the workflow to upgrade Doxygen
versions. This document describes the general steps in upgrading Drake to a
newer Doxygen.

See https://drake.mit.edu/documentation_instructions.html for instructions on
setting up a development environment to preview the site. In general, only the
`//doc/doxygen_cxx:build` target should be relevant for updating Doxygen, which
is faster to preview locally than the entire site.

# Upgrade Process

Use the following steps as a rough guide when upgrading to a new version of
Doxygen. This document is not meant to be a comprehensive set of all steps, but
an outline of some tools and knowledge that can be useful.

Before updating, it's a good idea to check the
[changelog](https://www.doxygen.nl/manual/changelog.html) for any new features
or fixes. There might be new things to be taken advantage of or keep up with
that aren't immediately obvious from just trudging along with the upgrade.

This process is only supported on Ubuntu 24.04 "Noble".

## Updating the Binary

1. Download the official source code of the latest release from
   https://github.com/doxygen/doxygen/releases. Upload the source tarball
   directly to S3 in the bucket defined in `//tools/workspace/mirrors.bzl`.
2. Update the `Dockerfile` in this directory. At minimum, any version-specific
   fields such as `DOXYGEN_VERSION` and `DOXYGEN_SHA256` should be updated.
   `UBUNTU_CODENAME` may also be updated to the primary developer platform at
   the time of running (currently Ubuntu 24.04 "Noble").
3. Run `tools/workspace/doxygen_internal/build_binaries_with_docker`. In short,
   this downloads, untars, and builds the previously-uploaded sources and should
   produce a tarball with a working binary inside.
    * At this point, as a sanity check, it might not be a bad idea to untar
      the produced tarball and run `./doxygen --version`, just to make sure
      everything worked as expected.
    * It will be convenient to keep this tarball saved for local development
      throughout the next few steps.
4. Update the `repository.bzl` in this directory to refer to the updated name
   and SHA.

At this point, the updated `doxygen` binary will be pulled down during
documentation generation.

## Updating Core Doxygen Files in Drake

These steps will utilize the new `doxygen` binary saved from the previous
section.

5. Run `./doxygen -s -u doc/doxygen_cxx/Doxyfile_CXX.in` to have Doxygen
   automatically update the the configuration file with any new fields. (This
   will keep any of Drake's previous values intact, but will add any new
   configuration fields with their default values.)
    * Closely examine the new fields and their behavior; some will likely be
      useless for Drake, but some could have significant implications. See
      https://www.doxygen.nl/manual/config.html for details.
    * Update the version at the top of the file; this is inserted manually by
      the project maintainers for clarity and history.
    * Remove any spurious changes, such as removed comments, quotes, etc.
6. Run `./doxygen -l` to generate a new `DoxygenLayout.xml` file. Drake's copy
   currently lives in `doc/doxygen_cxx/DoxygenLayout.xml`. Unlike the
   `Doxyfile_CXX.in`, there is no "update" command for this file to maintain
   old customizations/fields.
    * Closely review the diff between the old and new files and accept or reject
      changes to behavior on a case-by-case basis.
    * Similar to the previous step, update the version atop the file.

## Updates to Styling

Drake's website has many customizations applied on top of Doxygen which may
need adjusting to match upstream changes.

7. If there is an error in applying `header.html.patch`, that is because Drake's
   `doc/doxygen_cxx/header.html.patch` file needs updating. Navigate to
   the sources of the given release and find the `header.html` file; this is the
   file being patched. It may just be that the fuzz / line numbers need to
   be adopted.
8. Drake's custom CSS is found at `doc/doxygen_cxx/doxygen_extra.css`. Examine
   the site for any style elements that don't look quite right with the new
   version; it's likely that some upstream element changed out from under us,
   and our customization needs to be adopted.

## Updates to Documentation Contents

In general, the process of ensuring there aren't any regressions with the
actual contents of the documentation is a manual comparison of the site between
the old and new versions. Scan the pages and look for any edge cases where
Doxygen's behavior may have changed.
