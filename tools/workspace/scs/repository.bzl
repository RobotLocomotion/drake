# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def scs_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "cvxgrp/scs",
        # When updating this commit, see drake/tools/workspace/qdldl/README.md.
        commit = "v2.1.2",
        sha256 = "b891c0d20e6ff1d7971add407f011b6ef9cd064c298c5662515f6d7875a06a90",  # noqa
        build_file = "@drake//tools/workspace/scs:package.BUILD.bazel",
        patches = [
            # Fix some include paths for our build of QDLDL.
            # TODO(jwnimmer-tri) We should upstream these options under a
            # config switch.
            "@drake//tools/workspace/scs:private.h.diff",
            # Fix sizeof(bool) for our build of QDLDL.
            # TODO(jwnimmer-tri) We should upstream this fix.
            "@drake//tools/workspace/scs:private.c.diff",
        ],
        mirrors = mirrors,
    )
