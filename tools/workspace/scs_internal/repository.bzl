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
        commit = "3.2.6",
        sha256 = "70b5423a6c1cce4fa510f1746803cb0922c51c88c1a9ad8bdb55c3537777bac2",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/include_paths.patch",
            ":patches/norm_inf.patch",
        ],
        mirrors = mirrors,
    )
