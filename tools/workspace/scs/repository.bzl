# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def scs_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "cvxgrp/scs",
        commit = "v2.0.2",
        sha256 = "8725291dfe952a1f117f1f725906843db392fe8d29eebd8feb14b49f25fc669e",  # noqa
        build_file = "@drake//tools/workspace/scs:package.BUILD.bazel",
        patches = [
            # Fix include paths to use the system version of suitesparse
            # instead of a vendored copy.  TODO(jwnimmer-tri) Ideally, we
            # should upstream this option (under a config switch) once we're on
            # the latest upstream (which don't use SuiteSparse LDL anymore).
            "@drake//tools/workspace/scs:private.h.diff",
        ],
        mirrors = mirrors,
    )
