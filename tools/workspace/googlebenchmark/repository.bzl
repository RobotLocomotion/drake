# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def googlebenchmark_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/benchmark",
        commit = "v1.5.3",
        sha256 = "e4fbb85eec69e6668ad397ec71a3a3ab165903abe98a8327db920b94508f720e",  # noqa
        mirrors = mirrors,
    )
