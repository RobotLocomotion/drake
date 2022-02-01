# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def scs_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "cvxgrp/scs",
        # When updating this commit, see drake/tools/workspace/qdldl/README.md.
        commit = "3.1.0",
        sha256 = "90a7e58364ed3ea3375945a7f6f013de81c46100df80664a18f1c9b45a56be9c",  # noqa
        build_file = "@drake//tools/workspace/scs:package.BUILD.bazel",
        patches = [
            # Fix some include paths for our build of QDLDL.
            # TODO(jwnimmer-tri) We should upstream these options under a
            # config switch.
            "@drake//tools/workspace/scs:private.h.diff",
        ],
        mirrors = mirrors,
    )
