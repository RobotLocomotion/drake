load("@drake//tools/workspace:github.bzl", "github_archive")

def nlohmann_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "nlohmann/json",
        commit = "a0c1318830519eac027a31edec1a99ce1ae5670e",
        sha256 = "cc6706bb5ef4cefe9f81515cb00833cdaf78728293f8e8b1b3f09f43eb9e23da",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
