# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "tinyobjloader/tinyobjloader",
        commit = "0fc802cf468d23b9d205890b76b268f61b948e6d",
        sha256 = "b5ed8714735423da545a11c00df4d62e9158bba133b0a936a992c1e8e7b01abf",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            # We select tinyobjloader's floating-point typedef using a patch
            # file instead of `defines = []` because tinyobjloader is a private
            # dependency of Drake and we don't want the definition to leak into
            # all target that consume Drake.
            ":double_precision.patch",
            # We replace tinyobjloader's implementation of float parsing with a
            # faster call to strtod_l.
            ":faster_float_parsing.patch",
        ],
    )
