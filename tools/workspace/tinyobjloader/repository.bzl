# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "tinyobjloader/tinyobjloader",
        commit = "a1e8bad32e1ccd26a7936c5354ecf856aec2cf59",
        sha256 = "1b23bb6feb24cc3faac8a31c8470d268a679be89f7761d6fb488897b2606ca33",  # noqa
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
