# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def googlebenchmark_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/benchmark",
        commit = "v1.6.1",
        sha256 = "6132883bc8c9b0df5375b16ab520fac1a85dc9e4cf5be59480448ece74b278d4",  # noqa
        mirrors = mirrors,
        patches = [
            ":patches/console_allocs.patch",
            ":patches/string_precision.patch",
        ],
    )
