# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ignitionrobotics/sdformat",
        commit = "sdformat12_12.0.0",
        sha256 = "b3f44b7bc4530ca40ca120489d791d8ee9295beb30a16406926fb2ae42b3fba8",  # noqa
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        patches = [
            "@drake//tools/workspace/sdformat:patches/console.patch",
            "@drake//tools/workspace/sdformat:patches/deprecation_unit_testing.patch",  # noqa
            "@drake//tools/workspace/sdformat:patches/fix_broken_config.patch",
            "@drake//tools/workspace/sdformat:patches/no_global_config.patch",
            "@drake//tools/workspace/sdformat:patches/no_urdf.patch",
        ],
        mirrors = mirrors,
    )
