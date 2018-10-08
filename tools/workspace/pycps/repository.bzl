# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pycps_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # PR DRAFT: Update this once upstream PR lands.
        repository = "EricCousineau-TRI/pycps",
        commit = "950204d236a6bbed101645ae31d4ad05516cf3bd",
        sha256 = "d7ad2454be456d0d47915dbd3802bd7da3e921068d63b794df1c6144c1238285",  # noqa
        local_repository_override = "/home/eacousineau/proj/tri/repo/externals/pycps",
        build_file = "@drake//tools/workspace/pycps:package.BUILD.bazel",
        mirrors = mirrors,
    )
