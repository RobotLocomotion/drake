load("//tools/workspace:github.bzl", "github_archive")

def googlebenchmark_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/benchmark",
        commit = "v1.9.1",
        sha256 = "32131c08ee31eeff2c8968d7e874f3cb648034377dfc32a4c377fa8796d84981",  # noqa
        mirrors = mirrors,
        patches = [
            ":patches/console_allocs.patch",
            ":patches/remove_overloaded_fixture_set_up.patch",
            ":patches/string_precision.patch",
        ],
    )
