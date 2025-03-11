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
        commit = "v0.10.0",
        sha256 = "c9ffd958fd1643ffb076abcc2a5f56b6c30d536e16cbfac0db55749a22c91def",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/allow_unused_must_use.patch",
            ":patches/extern_c.patch",
            ":patches/git_submodule.patch",
            ":patches/sdp.patch",
        ],
        mirrors = mirrors,
    )
