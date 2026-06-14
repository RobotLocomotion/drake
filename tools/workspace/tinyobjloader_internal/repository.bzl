load("//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "tinyobjloader/tinyobjloader",
        commit = "62ff207968f3dc14a64a1e2378dce67b760e7c4a",
        sha256 = "672aa67aae74ebf90edf493015d45010423135720ebbf86d8441abf19bb8f94c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            ":patches/upstream/in_memory_parsing_support.patch",
            ":patches/faster_float_parsing.patch",
            ":patches/default_texture_color.patch",
            ":patches/silence_materials.patch",
            ":patches/remove_msvc_intrin.patch",
        ],
    )
