load("//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "tinyobjloader/tinyobjloader",
        commit = "cc327eecf7f8f4139932aec8d75db2d091f412ef",
        sha256 = "f1f5cd9b603a12da7f76868ce1720e55198f28a9cc46f19db9e60fa8ab9d76e1",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            ":patches/faster_float_parsing.patch",
            ":patches/default_texture_color.patch",
        ],
    )
