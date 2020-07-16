# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def googlebenchmark_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/benchmark",
        commit = "v1.5.1",
        sha256 = "23082937d1663a53b90cb5b61df4bcc312f6dee7018da78ba00dd6bd669dfef2",  # noqa
        mirrors = mirrors,
        patches = [
            # Suppress warnings from compiling Google Benchmark due to the
            # deprecated CSVReporter.
            # See: https://github.com/google/benchmark/issues/927.
            "@drake//tools/workspace/googlebenchmark:no_deprecated_declarations.patch",  # noqa
        ],
    )
