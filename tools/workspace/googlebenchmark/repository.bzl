# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def googlebenchmark_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/benchmark",
        commit = "v1.5.2",
        sha256 = "dccbdab796baa1043f04982147e67bb6e118fe610da2c65f88912d73987e700c",  # noqa
        mirrors = mirrors,
        patches = [
            # Suppress warnings from compiling Google Benchmark due to the
            # deprecated CSVReporter.
            # See: https://github.com/google/benchmark/issues/927.
            "@drake//tools/workspace/googlebenchmark:no_deprecated_declarations.patch",  # noqa
        ],
    )
