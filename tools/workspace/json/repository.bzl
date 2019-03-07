# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def json_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "nlohmann/json",
        commit = "v3.5.0",
        sha256 = "e0b1fc6cc6ca05706cce99118a87aca5248bd9db3113e703023d23f044995c1d",  # noqa
        build_file = "@drake//tools/workspace/json:package.BUILD.bazel",
        mirrors = mirrors,
    )
