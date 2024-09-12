load("//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "tinyobjloader/tinyobjloader",
        commit = "50461d0e0a77c178bb478e9319d7de82f469a848",
        sha256 = "20fa366e665636818b77684db39966f05be44eabe0d298249e722a3e604d5a59",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            ":patches/faster_float_parsing.patch",
            ":patches/default_texture_color.patch",
            ":patches/silence_materials.patch",
        ],
    )
