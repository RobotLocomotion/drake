# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def scs_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "cvxgrp/scs",
        # When updating this commit, see drake/tools/workspace/qdldl/README.md.
        commit = "v3.0.0",
        sha256 = "95ab61495db72b18d6bb690cd2ae2ce88134d078c473da6cb8750857ea17f732",  # noqa
        build_file = "@drake//tools/workspace/scs:package.BUILD.bazel",
        patches = [
            # Fix some include paths for our build of QDLDL.
            # TODO(jwnimmer-tri) We should upstream these options under a
            # config switch.
            "@drake//tools/workspace/scs:private.h.diff",
        ],
        mirrors = mirrors,
    )
