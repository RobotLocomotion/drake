load("//tools/workspace:github.bzl", "github_archive")

def fcl_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "b03987c495b75b5d873f932b1ff3fc27c1494c87",
        sha256 = "b055315cad3394b7bb1db8eead56a9063e95ed6cb0f43e12b244ab0bd2ce5be8",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/cassert.patch",
        ],
        mirrors = mirrors,
    )
