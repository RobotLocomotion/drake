# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:github.bzl", "github_archive")

def stduuid_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "mariusbancila/stduuid",
        commit = "3afe7193facd5d674de709fccc44d5055e144d7a",
        sha256 = "e11f9bf30c7f9c03d8e9a3a3fd7fe016eb5d8d9b89a2fe2c11b5f049e1d97916",  # noqa
        build_file = "@drake//tools/workspace/stduuid:package.BUILD.bazel",
        mirrors = mirrors,
    )
