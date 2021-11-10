# -*- python -*-
load("@drake//tools/workspace:github.bzl", "github_archive")

def conex_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ToyotaResearchInstitute/conex",
        commit = "a3d606f0339fa189dd2f4fb63e4b2c7b4c7ac202",
        sha256 = "0183056bbf3ca139251b218eadf2f6c0130519299acb0d07b4d870e8e7dc2289",
        mirrors = mirrors,
    )
