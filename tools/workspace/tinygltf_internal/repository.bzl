load("@drake//tools/workspace:github.bzl", "github_archive")

def tinygltf_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "syoyo/tinygltf",
        commit = "v2.8.22",
        sha256 = "97c3eb1080c1657cd749d0b49af189c6a867d5af30689c48d5e19521e7b5a070",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/function_pointer.patch",
            ":patches/json.patch",
        ],
        mirrors = mirrors,
    )
