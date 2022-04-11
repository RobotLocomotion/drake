# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ignitionrobotics/sdformat",
        # Make sure to update the PROJECT_* version numbers in package.BUILD.bazel
        commit = "sdformat12_12.4.0",
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        sha256 = "1feaa2c8867f3dbb3ddc9de34218a27cda0b881432057744b9ef12d4f26604b4",  # noqa
        patches = [
            "@drake//tools/workspace/sdformat:patches/console.patch",
            "@drake//tools/workspace/sdformat:patches/deprecation_unit_testing.patch",  # noqa
            "@drake//tools/workspace/sdformat:patches/no_urdf.patch",
        ],
        mirrors = mirrors,
    )
