This directory contains logic to support the drake_bazel_installed use case,
where external users can consume Drake binary releases via Bazel.  Check the
https://github.com/RobotLocomotion/drake-external-examples example named
"drake_bazel_installed" for how to use this.

The implementation mechanism is a single repo.bzl file that the user loads into
their WORKSPACE, which is then able to rehydrate the @drake workspace based on
the installed file paths.

We need to consolidate all of the rules and data needed by repo.bzl into a
single file, because loading sibling files as part of repo.bzl execution from
the binary install tree is difficult.

The repo_template.bzl file in this directory is the starting point to create
repo.bzl.  The repo_gen.py script replaces some constants in that template with
new values during the Drake source build.  In particular:

(1) The drake*.BUILD.bazel files in this directory are repacked into the second
half of repo.bzl as data, to be rehydrated into the @drake workspace for use by
the user.

(2) The generate_installed_files_manifest transcribes the list of installed
file paths into a manifest.bzl file, which is used by both repo.bzl and
drake*.BUILD.bazel to understand the list of files.

By convention, drake*BUILD.bazel targets that are private scaffolding specific
to drake_bazel_installed are named using a leading dot (e.g., `.pydrake`), to
distinguish them from public labels that would appear in a Drake source build.
