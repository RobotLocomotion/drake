load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "3f3d98a366799900be044852dd36aa54d1391239",
        sha256 = "318b6146df2213113abdca214d336857e12bed8973146a2005912444bd2f10ce",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/cassert.patch",
        ],
        mirrors = mirrors,
    )
