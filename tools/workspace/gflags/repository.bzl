# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:github.bzl", "github_archive")

def gflags_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gflags/gflags",
        commit = "v2.2.2",
        sha256 = "34af2f15cf7367513b352bdcd2493ab14ce43692d2dcd9dfc499492966c64dcf",  # noqa
        mirrors = mirrors,
    )
