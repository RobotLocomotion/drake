# -*- python -*-
load("@drake//tools/workspace:github.bzl", "github_archive")

def conex_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ToyotaResearchInstitute/conex",
        commit = "b1558e6ab2f2176363ca4362b2ea11a34738a77d",
        sha256 = "ecedc117845ed4f7249102ed2bd9d5864006fd6074ab543eb8ee377a43303afb",
        mirrors = mirrors,
    )
