# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pygccxml_py_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "CastXML/pygccxml",
        commit = "v2.1.0",
        sha256 = "3d5161990eae0febb1f4bd088ea717de694a0407c46ccb19aa87e5eebd358b02",  # noqa
        build_file = "@drake//tools/workspace/pygccxml_py:package.BUILD.bazel",
        mirrors = mirrors,
    )
