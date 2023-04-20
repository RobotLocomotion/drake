load("//tools/workspace:github.bzl", "github_archive")

def sdformat_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "azeey/sdformat",
        commit = "34f71d7febd488a7558ba6d47b4aac704091bbdd",
        build_file = ":package.BUILD.bazel",
        sha256 = "88a5747237c68ea5b4b6391e9b8892e863be5ff1e320514779ac4821839dd02e",  # noqa
        patches = [
            ":patches/console.patch",
            ":patches/deprecation_unit_testing.patch",
            ":patches/no_global_config.patch",
            ":patches/no_urdf.patch",
        ],
        mirrors = mirrors,
    )
