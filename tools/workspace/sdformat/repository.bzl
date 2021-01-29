# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "osrf/sdformat",
        commit = "b9fc65767f03c838c3583862ccb30a2c149ce230",
        sha256 = "defcae0b5bded0584701981528c1f6808ae4a81528ce1a041b7a29951040e035",  # noqa
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
    )
