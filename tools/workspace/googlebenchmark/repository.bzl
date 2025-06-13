load("//tools/workspace:github.bzl", "github_archive")

def googlebenchmark_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/benchmark",
        commit = "v1.9.4",
        sha256 = "b334658edd35efcf06a99d9be21e4e93e092bd5f95074c1673d5c8705d95c104",  # noqa
        mirrors = mirrors,
        patches = [
            ":patches/console_allocs.patch",
            ":patches/remove_overloaded_fixture_set_up.patch",
            ":patches/string_precision.patch",
        ],
    )
