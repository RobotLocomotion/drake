# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ignitionrobotics/sdformat",
        commit = "sdformat11_11.2.2",
        sha256 = "d81ab4922c168f1c88cd8092559acc0be7766ab4c1c2cbe31e9eca6b29a2120b",  # noqa
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
    )
