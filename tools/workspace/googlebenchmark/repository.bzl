# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def googlebenchmark_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/benchmark",
        commit = "v1.5.0",
        sha256 = "3c6a165b6ecc948967a1ead710d4a181d7b0fbcaa183ef7ea84604994966221a",  # noqa
        mirrors = mirrors,
        patches = [
            # Suppress warnings from compiling Google Benchmark due to the
            # deprecated CSVReporter.
            # See: https://github.com/google/benchmark/issues/927.
            "@drake//tools/workspace/googlebenchmark:no_deprecated_declarations.patch",  # noqa
        ],
    )
