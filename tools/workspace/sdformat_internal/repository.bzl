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
        commit = "sdformat15_15.2.0",
        build_file = ":package.BUILD.bazel",
        sha256 = "fb391247ae3b02d2f727104ab0b23bc0a906a9d049498a75380f9e0a139f4805",  # noqa
        patches = [
            ":patches/upstream/pr1523.patch",
            ":patches/upstream/pr1524.patch",
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
