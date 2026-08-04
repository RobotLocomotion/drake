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
        upgrade_type = "tag",
        commit = "sdformat16_16.1.0",
        build_file = ":package.BUILD.bazel",
        sha256 = "f9ef71787b0055f7ee35a3b52f42a9d6397f0c2f21f9572957bc3f456e2ca674",  # noqa
        patches = [
            ":patches/upstream/never_destroyed.patch",
            ":patches/upstream/no_density_not_a_problem.patch",
            ":patches/upstream/support_drake_visual.patch",
            ":patches/console.patch",
            ":patches/deprecation_unit_testing.patch",
            ":patches/environment.patch",
            ":patches/no_global_config.patch",
            ":patches/no_share_path.patch",
            ":patches/no_urdf.patch",
        ],
        mirrors = mirrors,
    )
