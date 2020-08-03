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
        commit = "7.0.2",
        sha256 = "7697e022f9cdc4f90b5e0a409643faa2cde0a6312f85e575c8388a1913374de5",  # noqa
        build_file = "@drake//tools/workspace/fmt:package.BUILD.bazel",
        mirrors = mirrors,
    )
