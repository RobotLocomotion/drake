# -*- mode: python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def dreal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "dreal/dreal4",
        commit = "4.20.03.4",
        sha256 = "7d24855e0ea18592e3a1681dc226a43ac82e119b583884c24f722536654b84d7",  # noqa
        mirrors = mirrors,
    )
