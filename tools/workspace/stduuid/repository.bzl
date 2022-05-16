# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:github.bzl", "github_archive")

def stduuid_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "mariusbancila/stduuid",
        commit = "c1e1c144af92aab8fd2af9acb5bd1f8f0a6ce4d4",
        sha256 = "7af03981102d417ad5d175821b3842506314c0eb665c5a7259654ed3ff340def",  # noqa
        build_file = "@drake//tools/workspace/stduuid:package.BUILD.bazel",
        mirrors = mirrors,
    )
