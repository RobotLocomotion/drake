load("//tools/workspace:github.bzl", "github_archive")

def fcl_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "cb0221698a9e877144d6945026e5c3fceb10439f",
        sha256 = "aee010daaf2b1d34a7408c6bcf9c64a65ddc72c91e5828b2e0e78161836db6c0",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
