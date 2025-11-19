load("//tools/workspace:github.bzl", "github_archive")

def scs_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "cvxgrp/scs",
        upgrade_advice = """
        When updating this commit, see
        drake/tools/workspace/qdldl_internal/README.md.
        """,
        commit = "3.2.9",
        sha256 = "f3d9095fb01fd634d12ccbe6f79ed2acbb7101ad57b723157d44a49cbe187669",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/include_paths.patch",
        ],
        mirrors = mirrors,
    )
