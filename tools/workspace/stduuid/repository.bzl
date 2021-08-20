# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:github.bzl", "github_archive")

def stduuid_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "mariusbancila/stduuid",
        commit = "4959d46eb494f8eddd4a37a1c8c6434b41d56ca8",
        sha256 = "011514f3c4ef831d26fa11ef897c6d0b92a491987f3b15a5b21cd872cf9153b4",  # noqa
        build_file = "@drake//tools/workspace/stduuid:package.BUILD.bazel",
        mirrors = mirrors,
    )
