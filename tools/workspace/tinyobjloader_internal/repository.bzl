load("//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "tinyobjloader/tinyobjloader",
        commit = "fe9e7130a0eee720a28f39b33852108217114076",
        sha256 = "73d0c0760df70a30da3ded67a0a1d3ba241bfde10e8260c54207b48662860ad0",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            ":patches/upstream/in_memory_parsing_support.patch",
            ":patches/upstream/silence_materials.patch",
            ":patches/faster_float_parsing.patch",
            ":patches/default_texture_color.patch",
        ],
    )
