# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fmt_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "fmtlib/fmt",
        commit = "4.1.0",
        sha256 = "46628a2f068d0e33c716be0ed9dcae4370242df135aed663a180b9fd8e36733d",  # noqa
        build_file = "@drake//tools/workspace/fmt:package.BUILD.bazel",
        mirrors = mirrors,
    )
