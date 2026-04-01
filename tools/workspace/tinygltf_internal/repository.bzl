load("@drake//tools/workspace:github.bzl", "github_archive")

def tinygltf_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "syoyo/tinygltf",
        commit = "v3.0.0",
        sha256 = "806b0f1ba8007837fcd531e23872286f8a8870ee23275e1eb5304cdb48e4cb30",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/json.patch",
        ],
        mirrors = mirrors,
    )
