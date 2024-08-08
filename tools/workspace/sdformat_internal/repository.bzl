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
        commit = "sdformat14_14.5.0",
        build_file = ":package.BUILD.bazel",
        sha256 = "44ff4916be63cad987c8ff2c21b3473e1e79c85a9b4fb83029536ed0ab0403e2",  # noqa
        patches = [
            ":patches/console.patch",
            ":patches/deprecation_unit_testing.patch",
            ":patches/environment.patch",
            ":patches/no_global_config.patch",
            ":patches/no_urdf.patch",
        ],
        mirrors = mirrors,
    )
