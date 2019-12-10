This directory contains logic to support the drake_bazel_installed use case,
where external users can consume Drake binary releases via Bazel.

For details of use, see:
https://github.com/RobotLocomotion/drake-external-examples/tree/master/drake_bazel_installed

The implmentation mechanism is a single repo.bzl file that the user loads into
their WORKSPACE, which is then able to rehydrate the @drake workspace based on
the installed file paths.

The repo.bzl file in this directory is the first half of that file; the second
half is appended at build-time using repo_gen.py.

The drake*.BUILD.bazel files in this directory are repacked into the second
half of repo.bzl as data, to be rehydrated into the @drake workspace for use by
the user.

The repo_gen_manifest.bzl transcribes the list of installed file paths into a
_manifest.bzl file, which is used by both repo.bzl and drake*.BUILD.bazel to
understand the list of files.
