# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def googlebenchmark_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/benchmark",
        commit = "v1.5.6",
        sha256 = "789f85b4810d13ff803834ea75999e41b326405d83d6a538baf01499eda96102",  # noqa
        mirrors = mirrors,
    )
