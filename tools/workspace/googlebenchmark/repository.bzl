load("//tools/workspace:github.bzl", "github_archive")

def googlebenchmark_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/benchmark",
        commit = "v1.8.5",
        sha256 = "d26789a2b46d8808a48a4556ee58ccc7c497fcd4c0af9b90197674a81e04798a",  # noqa
        mirrors = mirrors,
        patches = [
            ":patches/console_allocs.patch",
            ":patches/remove_overloaded_fixture_set_up.patch",
            ":patches/string_precision.patch",
        ],
    )
