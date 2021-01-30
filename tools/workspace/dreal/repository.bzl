# -*- mode: python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def dreal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "dreal/dreal4",
        commit = "4.20.12.1",
        sha256 = "8a8a726a370700e386ee337adf4d15a6cdc882285f29a7b6afb8f4b05c5f13b9",  # noqa
        mirrors = mirrors,
    )
