load("//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "tinyobjloader/tinyobjloader",
        commit = "b5346fac929c8a862a1c1d4d89db5477d30b7e6f",
        sha256 = "e522910e22e62581431ee1b1dd63a2033ead2043c9d99681f56935ea30fbf5c9",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            ":patches/upstream/in_memory_parsing_support.patch",
            ":patches/upstream/silence_materials.patch",
            ":patches/faster_float_parsing.patch",
            ":patches/default_texture_color.patch",
        ],
    )
