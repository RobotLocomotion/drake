load("@drake//tools/workspace:github.bzl", "github_archive")

def tinygltf_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "syoyo/tinygltf",
        commit = "v2.9.6",
        sha256 = "ba2c47a095136bfc8a5d085421e60eb8e8df3bca4ae36eb395084c1b264c6927",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/json.patch",
        ],
        mirrors = mirrors,
    )
