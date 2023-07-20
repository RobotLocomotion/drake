load("@drake//tools/workspace:github.bzl", "github_archive")

def googlebenchmark_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/benchmark",
        commit = "v1.8.2",
        sha256 = "2aab2980d0376137f969d92848fbb68216abb07633034534fc8c65cc4e7a0e93",  # noqa
        mirrors = mirrors,
        patches = [
            ":patches/console_allocs.patch",
            ":patches/remove_overloaded_fixture_set_up.patch",
            ":patches/string_precision.patch",
        ],
    )
