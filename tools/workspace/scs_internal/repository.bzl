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
        upgrade_type = "release",
        commit = "3.3.1",
        sha256 = "99a1437b2508ed29933d259793a5745f29000fd8ec58f63a8f54a20006aacb86",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/include_paths.patch",
        ],
        mirrors = mirrors,
    )
