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
        commit = "sdformat16_16.0.1",
        build_file = ":package.BUILD.bazel",
        sha256 = "4fac898700afb2953af5f8ac6b0221e4d9bc1e460aac6d4b7a5c3699c456126c",  # noqa
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
