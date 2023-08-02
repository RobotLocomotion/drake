load("@drake//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_repository(
        name,
        mirrors = None):
    """The @tinyobjloader external is deprecated in Drake's WORKSPACE and will
    be removed on or after 2023-11-01.
    """
    github_archive(
        name = name,
        repository = "tinyobjloader/tinyobjloader",
        commit = "f5569db1ffb3b0222663ba38a7a9b3f6a461c469",
        sha256 = "f0a0f5ee4e1c7cc573fba9044c1dbed2f547b3ff058840fc8f83c6b7b79f2391",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            ":patches/double_precision.patch",
            "@drake//tools/workspace/tinyobjloader_internal:patches/faster_float_parsing.patch",  # noqa
            "@drake//tools/workspace/tinyobjloader_internal:patches/default_texture_color.patch",  # noqa
        ],
    )
