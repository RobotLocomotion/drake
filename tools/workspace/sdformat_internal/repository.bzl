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
        commit = "sdformat15_15.3.0",
        build_file = ":package.BUILD.bazel",
        sha256 = "b187da7a21a6e0f5e2a45f6b4026a1a69fd39e3f50de26f028313595a1c10cfd",  # noqa
        patches = [
            ":patches/upstream/pr1522.patch",
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
