load("@drake//tools/workspace:github.bzl", "github_archive")

def tinygltf_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "syoyo/tinygltf",
        commit = "v2.9.3",
        sha256 = "f5f282508609a0098048c8ff25d72f4ef0995bc1d46bc7a5d740e559d80023d2",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/json.patch",
        ],
        mirrors = mirrors,
    )
