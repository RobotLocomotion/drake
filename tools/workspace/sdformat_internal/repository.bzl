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
        commit = "sdformat15_15.1.1",
        build_file = ":package.BUILD.bazel",
        sha256 = "9c9b517b390fe6965f76775db67f406b42948ebac433f87fadc680467d45184a",  # noqa
        patches = [
            ":patches/upstream/root_model_mutable_getter.patch",
            ":patches/upstream/specific_iostream_includes.patch",
            ":patches/upstream/support_drake_visual.patch",
            ":patches/upstream/version_singleton.patch",
            ":patches/console.patch",
            ":patches/deprecation_unit_testing.patch",
            ":patches/environment.patch",
            ":patches/no_global_config.patch",
            ":patches/no_share_path.patch",
            ":patches/no_urdf.patch",
        ],
        mirrors = mirrors,
    )
