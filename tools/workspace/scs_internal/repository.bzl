load("//tools/workspace:github.bzl", "github_archive")

def scs_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "cvxgrp/scs",
        upgrade_advice = """
        When updating this commit, see drake/tools/workspace/qdldl/README.md.
        """,
        commit = "3.2.7",
        sha256 = "bc8211cfd213f3117676ceb7842f4ed8a3bc7ed9625c4238cc7d83f666e22cc9",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/include_paths.patch",
            ":patches/norm_inf.patch",
        ],
        mirrors = mirrors,
    )
