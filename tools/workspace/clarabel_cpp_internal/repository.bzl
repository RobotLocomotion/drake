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
        commit = "v0.9.0",
        sha256 = "4dfff59667623bf4264ba27f25eef08bda64899a4e443f8d1993d5d5a9a36ab0",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/extern_c.patch",
            ":patches/git_submodule.patch",
            ":patches/sdp.patch",
            ":patches/upstream/allow_unused_must_use.patch",
        ],
        mirrors = mirrors,
    )
