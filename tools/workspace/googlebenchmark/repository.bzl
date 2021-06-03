# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def googlebenchmark_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/benchmark",
        commit = "v1.5.4",
        sha256 = "e3adf8c98bb38a198822725c0fc6c0ae4711f16fbbf6aeb311d5ad11e5a081b5",  # noqa
        mirrors = mirrors,
    )
