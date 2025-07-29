load("//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "tinyobjloader/tinyobjloader",
        commit = "a4e519b0a0f29c790464fcfeadfe25a7f9fa15ff",
        sha256 = "3a900a7e087830b3384c13684daa5d0667016be7b1c0ab906c83a29f78b312a2",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            ":patches/upstream/in_memory_parsing_support.patch",
            ":patches/upstream/silence_materials.patch",
            ":patches/faster_float_parsing.patch",
            ":patches/default_texture_color.patch",
        ],
    )
