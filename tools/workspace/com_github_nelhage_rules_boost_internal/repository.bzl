load("//tools/workspace:github.bzl", "github_archive")

def com_github_nelhage_rules_boost_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "nelhage/rules_boost",
        commit = "d8f4f9f88b12461b19354dea4df5f9ee78262067",
        sha256 = "178680b0a5093e17a747bfe8860adc7d5c32a9105983c1075478210973d1b083",  # noqa
        patches = [
            ":patches/iostreams.patch",
        ],
        mirrors = mirrors,
    )
