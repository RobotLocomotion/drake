# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "tinyobjloader/tinyobjloader",
        commit = "8322e00ae685ea623ab6ac5a6cebcfa2d22fbf93",
        sha256 = "c7596003a94bd73ec0d9f1de3f26595902eb0db5291c2f89932264ee63fb0492",  # noqa
        build_file = "@drake//tools/workspace/tinyobjloader:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
        patches = [
            # We select tinyobjloader's floating-point typedef using a patch
            # file instead of `defines = []` because tinyobjloader is a private
            # dependency of Drake and we don't want the definition to leak into
            # all target that consume Drake.
            "@drake//tools/workspace/tinyobjloader:double_precision.patch",
            # We replace tinyobjloader's implementation of float parsing with a
            # faster call to strtod_l.
            "@drake//tools/workspace/tinyobjloader:faster_float_parsing.patch",
        ],
    )
