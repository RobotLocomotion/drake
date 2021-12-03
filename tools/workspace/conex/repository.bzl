# -*- python -*-
load("@drake//tools/workspace:github.bzl", "github_archive")

def conex_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ToyotaResearchInstitute/conex",
        commit = "164a33df764d1f458756643e010f1bb62d0e1479",
        sha256 = "3f88f45276a1b474946b67e7c650fefd6d7c9dcb48f0c0a11393be3e6adc5ba7",  # noqa
        build_file = "@drake//tools/workspace/conex:package.BUILD.bazel",
        mirrors = mirrors,
    )
