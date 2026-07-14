load("//tools/workspace:github.bzl", "github_archive")

def nanobind_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "wjakob/nanobind",
        upgrade_type = "release",
        commit = "v2.13.0",
        sha256 = "cb25a582ccade4b6067bc73c78b84ad9dbd0bbe0e537320711d18015ccafc4ef",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/ndarray_extra_import.patch",
        ],
        mirrors = mirrors,
    )
