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
        commit = "5.3.0",
        sha256 = "defa24a9af4c622a7134076602070b45721a43c51598c8456ec6f2c4dbb51c89",  # noqa
        build_file = "@drake//tools/workspace/fmt:package.BUILD.bazel",
        mirrors = mirrors,
    )
