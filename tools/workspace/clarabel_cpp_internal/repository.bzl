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
        commit = "v0.11.1",
        sha256 = "efa90703958075cc59afe8f54ca164eff057f0fcf9877291bbef11cf960f80c8",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/extern_c.patch",
            ":patches/git_submodule.patch",
            ":patches/sdp.patch",
        ],
        mirrors = mirrors,
    )
