load("//tools/workspace:github.bzl", "github_archive")

def implib_so_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "yugr/Implib.so",
        commit = "ecf7bb51a92a0fb16834c5b698570ab25f9f1d21",
        sha256 = "f5d657ec0b6361364043d932056a9c49cf682c0aaacc6c42ea40c166cab7ef21",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
