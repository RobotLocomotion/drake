load("@drake//tools/workspace:github.bzl", "github_archive")

def tinygltf_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "syoyo/tinygltf",
        upgrade_type = "release",
        commit = "v3.0.1",
        sha256 = "b7e953f13a30d7b6fd677e484de35febde954143a265da15dacd92ed171c73e6",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/never_destroyed.patch",
        ],
        mirrors = mirrors,
    )
