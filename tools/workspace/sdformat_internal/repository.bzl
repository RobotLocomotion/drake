# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gazebosim/sdformat",
        commit = "sdformat13_13.0.0",
        build_file = ":package.BUILD.bazel",
        sha256 = "87027e1cab8bc88160cbf1437c8e7f1604f38a0400799d553ab1b47804abcad6",  # noqa
        patches = [
            ":patches/console.patch",
            ":patches/deprecation_unit_testing.patch",
            ":patches/no_global_config.patch",
            ":patches/no_urdf.patch",
        ],
        mirrors = mirrors,
    )
