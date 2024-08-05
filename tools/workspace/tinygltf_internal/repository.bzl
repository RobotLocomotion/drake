load("@drake//tools/workspace:github.bzl", "github_archive")

def tinygltf_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "syoyo/tinygltf",
        commit = "v2.9.2",
        sha256 = "b34d1456bb1d63bbb4e05ea1e4d8691d0253a03ef72385a8bffd2fae4b743feb",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/json.patch",
        ],
        mirrors = mirrors,
    )
