# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gazebosim/sdformat",
        commit = "sdformat13_13.0.1",
        build_file = ":package.BUILD.bazel",
        sha256 = "4dfb9d938f77d590c4465c9ad5769777e5b0f40dd21c288fe2cc1987d5a0b036",  # noqa
        patches = [
            ":patches/console.patch",
            ":patches/deprecation_unit_testing.patch",
            ":patches/no_global_config.patch",
            ":patches/no_urdf.patch",
        ],
        mirrors = mirrors,
    )
