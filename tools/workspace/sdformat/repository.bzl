# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ignitionrobotics/sdformat",
        commit = "sdf12",
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        sha256 = "9cadc96af2d3823725c9ab3f6a3c7c9bd325ec49367c84dfd97471aebd6a18d1",  # noqa
        patches = [
            "@drake//tools/workspace/sdformat:patches/console.patch",
        ],
        local_repository_override = "/home/addisu/ws/fortress/src/sdformat",
        mirrors = mirrors,
    )
