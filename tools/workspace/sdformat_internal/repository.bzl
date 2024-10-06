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
        commit = "sdformat15_15.0.0",
        build_file = ":package.BUILD.bazel",
        sha256 = "2b21c1efe7c4bb817a5867b80974c7e35cfdd863b4024fe94a3bcb1d6a05a813",  # noqa
        patches = [
            ":patches/console.patch",
            ":patches/deprecation_unit_testing.patch",
            ":patches/environment.patch",
            ":patches/no_global_config.patch",
            ":patches/no_share_path.path",
            ":patches/no_urdf.patch",
        ],
        mirrors = mirrors,
    )
