# -*- python -*-

# This dependency is part of a "cohort" defined in
# drake/tools/workspace/new_release.py.  This dependency should only be
# updated in conjunction with the other members of its cohort.

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gazebosim/sdformat",
        commit = "sdformat13_13.2.0",
        build_file = ":package.BUILD.bazel",
        sha256 = "e1084091f65caf8aabd4cfca6df3a7bf3a9563de1715829810813840598d5de3",  # noqa
        patches = [
            ":patches/console.patch",
            ":patches/deprecation_unit_testing.patch",
            ":patches/no_global_config.patch",
            ":patches/no_urdf.patch",
        ],
        mirrors = mirrors,
    )
