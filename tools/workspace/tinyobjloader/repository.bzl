# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "tinyobjloader/tinyobjloader",
        commit = "9173980d1de273b17eba5e10eb189e8b4be89425",
        sha256 = "fe06bf462bf386ea7f6bf34f94c78099c849df348d4a6df681707fba5b5b643c",  # noqa
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
