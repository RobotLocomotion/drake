# -*- python -*-
load("@drake//tools/workspace:github.bzl", "github_archive")

def conex_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ToyotaResearchInstitute/conex",
        commit = "99c2fbba3339473681629848d62e31278349c004",
        sha256 = "2e1a33d5fd27545c4f27b47d1710534b13d08245e9e73835e4741277a2953c8d",  # noqa
        mirrors = mirrors,
    )
