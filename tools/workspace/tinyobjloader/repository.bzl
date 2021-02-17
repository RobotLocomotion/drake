# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "calderpg-tri/tinyobjloader",
        commit = "129a88b40f115951163c284dc9d62de5bbfc3bd1",
        sha256 = "96bb917d81ac7fbfeb391bc39a7cc5e6a409a12a39fbf5e7169189dfdd7ef284",  # noqa
        build_file = "@drake//tools/workspace/tinyobjloader:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
        patches = [
            # We select tinyobjloader's floating-point typedef using a patch
            # file instead of `defines = []` because tinyobjloader is a private
            # dependency of Drake and we don't want the definition to leak into
            # all target that consume Drake.
            "@drake//tools/workspace/tinyobjloader:double_precision.patch",
        ],
    )
