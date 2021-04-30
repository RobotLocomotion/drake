# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def scs_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "cvxgrp/scs",
        # When updating this commit, see drake/tools/workspace/qdldl/README.md.
        commit = "v2.1.3",
        sha256 = "cb139aa8a53b8f6a7f2bacec4315b449ce366ec80b328e823efbaab56c847d20",  # noqa
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
