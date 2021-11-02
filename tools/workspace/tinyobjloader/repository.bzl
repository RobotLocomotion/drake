# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "tinyobjloader/tinyobjloader",
        commit = "097ad6a826336a76e42a27e6e531da7dbdf3fd01",
        sha256 = "cc326cc880620ad68f04e5a662ace6da5adbd70f03569c855651e95ba66ca06c",  # noqa
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
