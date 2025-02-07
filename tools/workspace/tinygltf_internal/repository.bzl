load("@drake//tools/workspace:github.bzl", "github_archive")

def tinygltf_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "syoyo/tinygltf",
        commit = "v2.9.5",
        sha256 = "7b93da27c524dd17179a0eeba6f432b0060d82f6222630ba027c219ce11e24db",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/json.patch",
        ],
        mirrors = mirrors,
    )
