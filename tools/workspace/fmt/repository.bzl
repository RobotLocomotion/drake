# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fmt_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "fmtlib/fmt",
        # When changing the fmt version, also update the URL in the file
        # overview docstring of drake/common/text_logging.h.
        commit = "7.0.1",
        sha256 = "ac335a4ca6beaebec4ddb2bc35b9ae960b576f3b64a410ff2c379780f0cd4948",  # noqa
        build_file = "@drake//tools/workspace/fmt:package.BUILD.bazel",
        mirrors = mirrors,
    )
