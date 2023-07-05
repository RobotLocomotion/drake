load("//tools/workspace:github.bzl", "github_archive")

def sdformat_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "gazebosim/sdformat",
        commit = "sdformat13_13.5.0",
        build_file = ":package.BUILD.bazel",
        sha256 = "a8e9adede81b34564a47e074d26d5d97b1906db76e0ed0da17af72ed45e79dec",  # noqa
        patches = [
            ":patches/console.patch",
            ":patches/deprecation_unit_testing.patch",
            ":patches/no_global_config.patch",
            ":patches/no_urdf.patch",
        ],
        mirrors = mirrors,
    )
