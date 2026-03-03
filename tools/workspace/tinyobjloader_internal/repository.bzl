load("//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "tinyobjloader/tinyobjloader",
        commit = "a285d14032202d07a210f98176fe4e9021b39c19",
        sha256 = "36ec033be72be865547536895a3f38271f6ddd3abaea27ea6666880111671a54",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            ":patches/upstream/in_memory_parsing_support.patch",
            ":patches/upstream/silence_materials.patch",
            ":patches/faster_float_parsing.patch",
            ":patches/default_texture_color.patch",
        ],
    )
