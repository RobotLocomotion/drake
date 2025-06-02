load("@drake//tools/workspace:github.bzl", "github_archive")

def clarabel_cpp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "oxfordcontrol/Clarabel.cpp",
        commit = "v0.11.0",
        sha256 = "a4a8002c4b98b96bf213e5698c9bfc63ef269c9f9a1a2c42fbb0e766f07ee3ec",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/extern_c.patch",
            ":patches/git_submodule.patch",
            ":patches/sdp.patch",
        ],
        mirrors = mirrors,
    )
