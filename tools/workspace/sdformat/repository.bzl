# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def sdformat_repository(
        name,
        mirrors = None):
    bitbucket_archive(
        name = name,
        repository = "osrf/sdformat",
        commit = "609fefa24699",
        sha256 = "83b3d077318ad387edcf374ecacd01c919e42917897ec944f13e9b5669474353",  # noqa
        strip_prefix = "osrf-sdformat-609fefa24699",
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
    )
