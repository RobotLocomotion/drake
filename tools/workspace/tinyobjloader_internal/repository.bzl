load("//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "tinyobjloader/tinyobjloader",
        commit = "3bb554cf74428d7db13418b4aca1b9752a1d2be8",
        sha256 = "81eaf7bfff527904d1d1320bd7a6b1577379458b57ecd2f496c52396b5034292",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            ":patches/upstream/in_memory_parsing_support.patch",
            ":patches/upstream/silence_materials.patch",
            ":patches/faster_float_parsing.patch",
            ":patches/default_texture_color.patch",
        ],
    )
