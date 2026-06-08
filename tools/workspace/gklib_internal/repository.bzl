load("//tools/workspace:github.bzl", "github_archive")

def gklib_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "KarypisLab/GKlib",
        commit = "3b7d61b9f885063c89901f3901fb4426f9cfb58f",
        sha256 = "ce44d6d64a786df09dc979b0b18f095b0fd95d03682bdaac5276e4d234b41b56",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
