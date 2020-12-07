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
        commit = "7.1.3",
        sha256 = "5cae7072042b3043e12d53d50ef404bbb76949dad1de368d7f993a15c8c05ecc",  # noqa
        build_file = "@drake//tools/workspace/fmt:package.BUILD.bazel",
        mirrors = mirrors,
    )
