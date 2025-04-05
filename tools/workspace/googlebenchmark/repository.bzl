load("//tools/workspace:github.bzl", "github_archive")

def googlebenchmark_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/benchmark",
        commit = "v1.9.2",
        sha256 = "409075176168dc46bbb81b74c1b4b6900385b5d16bfc181d678afb060d928bd3",  # noqa
        mirrors = mirrors,
        patches = [
            ":patches/console_allocs.patch",
            ":patches/remove_overloaded_fixture_set_up.patch",
            ":patches/string_precision.patch",
        ],
    )
