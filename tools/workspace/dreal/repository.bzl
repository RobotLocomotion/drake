# -*- mode: python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def dreal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "dreal/dreal4",
        commit = "4.20.09.1",
        sha256 = "e5fa7c0acd6e0f259507476fd58973e3f5ef0baa741a7a34909076aebd25044a",  # noqa
        mirrors = mirrors,
    )
