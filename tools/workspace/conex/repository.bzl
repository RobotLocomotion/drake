# -*- python -*-
load("@drake//tools/workspace:github.bzl", "github_archive")

def conex_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ToyotaResearchInstitute/conex",
        commit = "2a87558fa5cf1fdc23bf36d1038aab79f8a28a67",
        sha256 = "5fa479b38bb28c004946c23ef1767cbf0ff646c8ca8f5df4f1821cb4db743395",  # noqa
        mirrors = mirrors,
    )
