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
        commit = "sdformat14_14.1.1",
        build_file = ":package.BUILD.bazel",
        sha256 = "50f2b64b425e0810f20de0640adad9aaa15ccb95d897b2122033bb4d02d93929",  # noqa
        patches = [
            ":patches/console.patch",
            ":patches/deprecation_unit_testing.patch",
            ":patches/environment.patch",
            ":patches/no_global_config.patch",
            ":patches/no_urdf.patch",
        ],
        mirrors = mirrors,
    )
