# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def scs_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "cvxgrp/scs",
        # When updating this commit, see drake/tools/workspace/qdldl/README.md.
        commit = "2.1.1",
        sha256 = "0e20b91e8caf744b84aa985ba4e98cc7235ee33612b2bad2bf31ea5ad4e07d93",  # noqa
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
