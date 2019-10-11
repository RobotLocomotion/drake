# -*- mode: python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def dreal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "dreal/dreal4",
        commit = "4.19.10.2",
        sha256 = "0488b4da501bace1fc2e2418ce7f8b1d28b6bd798923c2619756a5e905cd83a5",  # noqa
        mirrors = mirrors,
    )
