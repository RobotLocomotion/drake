# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gazebosim/sdformat",
        commit = "sdformat12_12.5.0",
        build_file = "@drake//tools/workspace/sdformat_internal:package.BUILD.bazel",  # noqa
        sha256 = "3896772db68b7ca7b18bbf1945a72206885b03d3f0caf29491be5b53b79a7124",  # noqa
        patches = [
            "@drake//tools/workspace/sdformat_internal:patches/1043.patch",
            "@drake//tools/workspace/sdformat_internal:patches/console.patch",
            "@drake//tools/workspace/sdformat_internal:patches/deprecation_unit_testing.patch",  # noqa
            "@drake//tools/workspace/sdformat_internal:patches/no_global_config.patch",  # noqa
            "@drake//tools/workspace/sdformat_internal:patches/no_urdf.patch",
        ],
        mirrors = mirrors,
    )
