# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:github.bzl", "github_archive")

def stduuid_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "mariusbancila/stduuid",
        commit = "3dca9d816cce4f142e82fe249d71c4f781bb7aee",
        sha256 = "9adb87f44672a81525aa45ea390156c37361808f14fbc86840b3aca7d64a17a2",  # noqa
        build_file = "@drake//tools/workspace/stduuid:package.BUILD.bazel",
        mirrors = mirrors,
    )
