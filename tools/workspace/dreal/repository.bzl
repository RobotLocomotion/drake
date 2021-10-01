# -*- mode: python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def dreal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "dreal/dreal4",
        commit = "4.21.06.2",
        sha256 = "7bbd328a25c14cff814753694b1823257bb7cff7f84a7b705b9f111624d5b2e4",  # noqa
        mirrors = mirrors,
        patches = [
            "@drake//tools/workspace/dreal:ibex_2.8.6.patch",
        ],
    )
