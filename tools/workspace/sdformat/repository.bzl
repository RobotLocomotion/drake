# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gazebosim/sdformat",
        # Make sure to update the PROJECT_* version numbers in
        # package.BUILD.bazel
        commit = "sdformat12_12.5.0",
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        sha256 = "3896772db68b7ca7b18bbf1945a72206885b03d3f0caf29491be5b53b79a7124",  # noqa
        patches = [
            "@drake//tools/workspace/sdformat:patches/console.patch",
            "@drake//tools/workspace/sdformat:patches/deprecation_unit_testing.patch",  # noqa
            "@drake//tools/workspace/sdformat:patches/no_global_config.patch",
            "@drake//tools/workspace/sdformat:patches/no_urdf.patch",
        ],
        mirrors = mirrors,
    )
