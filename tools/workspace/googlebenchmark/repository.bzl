# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def googlebenchmark_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/benchmark",
        commit = "v1.6.0",
        sha256 = "1f71c72ce08d2c1310011ea6436b31e39ccab8c2db94186d26657d41747c85d6",  # noqa
        mirrors = mirrors,
    )
