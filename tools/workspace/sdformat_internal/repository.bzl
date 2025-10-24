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
        commit = "sdformat16_16.0.0",
        build_file = ":package.BUILD.bazel",
        sha256 = "ff70394ec4ff730aead09b5f7d6529588eadfee2da05a3501d99dcf2e1d0d593",  # noqa
        patches = [
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
