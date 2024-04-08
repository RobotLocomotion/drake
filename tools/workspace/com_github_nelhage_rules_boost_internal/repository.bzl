load("//tools/workspace:github.bzl", "github_archive")

def com_github_nelhage_rules_boost_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "nelhage/rules_boost",
        commit = "52dfb0c28241d48047fe59dfcea92f9370099a13",
        sha256 = "3fe1fa91450e47b0c969640ec0d7a6495c5ea310538d6316b0787cd01087391c",  # noqa
        patches = [
            ":patches/iostreams.patch",
        ],
        mirrors = mirrors,
    )
