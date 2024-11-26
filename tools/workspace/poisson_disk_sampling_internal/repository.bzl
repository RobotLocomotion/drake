load("//tools/workspace:github.bzl", "github_archive")

def poisson_disk_sampling_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "thinks/poisson-disk-sampling",
        commit = "v0.4.0",
        sha256 = "a1bae19286d2037f7dd23e554e37f3f44095bc2a1be961261a665d37fd1473ba",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
