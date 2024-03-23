load("//tools/workspace:github.bzl", "github_archive")

def com_github_nelhage_rules_boost_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "nelhage/rules_boost",
        commit = "00b9b9ecb9b43564de44ea0b10e22b29dcf84d79",
        sha256 = "a8499f581899ae7356e40e2aab6e985dd2da9c894c91197341aace9a0a6157fe",  # noqa
        patches = [
            ":patches/iostreams.patch",
        ],
        mirrors = mirrors,
    )
