# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def json_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "nlohmann/json",
        commit = "v3.4.0",
        sha256 = "c377963a95989270c943d522bfefe7b889ef5ed0e1e15d535fd6f6f16ed70732",  # noqa
        build_file = "@drake//tools/workspace/json:package.BUILD.bazel",
        mirrors = mirrors,
    )
