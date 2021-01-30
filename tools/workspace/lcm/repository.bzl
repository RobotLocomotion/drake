# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        commit = "501bb446d42c9a57f9e5ddf3c41ba78f4735c9f2",
        sha256 = "e6a40dcb3e77f8eb27c11aa8dd7c4edb9784e1928f617db395865a5a94f81377",  # noqa
        build_file = "@drake//tools/workspace/lcm:package.BUILD.bazel",
        mirrors = mirrors,
    )
