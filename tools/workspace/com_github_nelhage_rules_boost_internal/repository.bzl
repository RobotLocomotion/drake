load("//tools/workspace:github.bzl", "github_archive")

def com_github_nelhage_rules_boost_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "nelhage/rules_boost",
        commit = "f621ad7bec2abf5a597ed1271fd823d2761943b2",
        sha256 = "eb2cadbeb990785d8004d7063cac5fba72518cee7bdd2b9f7597affdced7524e",  # noqa
        patches = [
            ":patches/iostreams.patch",
        ],
        mirrors = mirrors,
    )
