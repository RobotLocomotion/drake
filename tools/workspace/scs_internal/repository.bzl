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
        commit = "3.2.10",
        sha256 = "99c61af0e2cc50aec9f03c14a4d2d73492f00f5df6e127420b48fc2a9fc1392b",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/include_paths.patch",
        ],
        mirrors = mirrors,
    )
