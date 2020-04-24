# -*- mode: python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def dreal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "dreal/dreal4",
        commit = "4.20.04.1",
        sha256 = "25f5bc0c15f902882fad8bb5ac0c432efddf974766a18f9ccb964c2907674806",  # noqa
        mirrors = mirrors,
    )
