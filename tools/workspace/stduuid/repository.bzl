# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:github.bzl", "github_archive")

def stduuid_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "mariusbancila/stduuid",
        commit = "5c538cca02932aa0266659661d5b4726f3a317c7",
        sha256 = "95396571d921e605e6f05a911de64ae1eec87a45d28eb6db4e68710bdf28f065",  # noqa
        build_file = "@drake//tools/workspace/stduuid:package.BUILD.bazel",
        mirrors = mirrors,
    )
