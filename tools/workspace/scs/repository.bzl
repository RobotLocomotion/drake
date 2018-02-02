# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def scs_repository(name):
    github_archive(
        name = name,
        repository = "cvxgrp/scs",
        commit = "v1.2.6",
        sha256 = "b4bebb43a1257b6e88a5f97c855c0559d6c8a8c0548d3156fc5a28d82bb9533f",  # noqa
        build_file = "@drake//tools/workspace/scs:package.BUILD.bazel",
    )
