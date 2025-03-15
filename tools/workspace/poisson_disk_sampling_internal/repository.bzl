load("//tools/workspace:github.bzl", "github_archive")

def poisson_disk_sampling_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "thinks/tph_poisson",
        commit = "v0.4.0",
        sha256 = "3be4a705ca234ec0dbcc8115e8ac31e9f4dae02423e07e75929d75ed70db0a2d",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/positive_capacity.patch",
        ],
        mirrors = mirrors,
    )
