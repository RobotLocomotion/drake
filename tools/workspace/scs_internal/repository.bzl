load("@drake//tools/workspace:github.bzl", "github_archive")

def scs_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "cvxgrp/scs",
        upgrade_advice = """
        When updating this commit, see drake/tools/workspace/qdldl/README.md.
        """,
        commit = "3.2.3",
        sha256 = "fe5e8c61ca5ea97975e231b1bb4a873d86e7908fdff416101c2a7cd13ecf5b41",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            # Fix some include paths for our build of QDLDL.
            # TODO(jwnimmer-tri) We should upstream these options under a
            # config switch.
            ":private.h.diff",
        ],
        mirrors = mirrors,
    )
