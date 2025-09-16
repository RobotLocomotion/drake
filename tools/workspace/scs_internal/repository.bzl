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
        commit = "3.2.8",
        sha256 = "22d2d785b7c7a9ee8a260d2684cf17ae4733271b8421fdbc78f281d19910ca1b",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/include_paths.patch",
        ],
        mirrors = mirrors,
    )
