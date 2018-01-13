# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def sdformat_repository(name):
    bitbucket_archive(
        name = name,
        repository = "osrf/sdformat",
        commit = "bac3dfb42cc7",
        sha256 = "212211eddd9fa010b4b61a2dae87cd84a66a8b78ed302612d214b7388f9bc198",  # noqa
        strip_prefix = "osrf-sdformat-bac3dfb42cc7",
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
    )
