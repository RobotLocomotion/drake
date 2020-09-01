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
        commit = "7.0.3",
        sha256 = "b4b51bc16288e2281cddc59c28f0b4f84fed58d016fb038273a09f05f8473297",  # noqa
        build_file = "@drake//tools/workspace/fmt:package.BUILD.bazel",
        mirrors = mirrors,
    )
