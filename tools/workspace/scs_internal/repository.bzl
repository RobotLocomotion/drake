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
        commit = "3.2.11",
        sha256 = "ceb5d9ecf35836ee7e0ce64566190f11a99314ec8143dbb909329809afa3f77f",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/include_paths.patch",
        ],
        mirrors = mirrors,
    )
