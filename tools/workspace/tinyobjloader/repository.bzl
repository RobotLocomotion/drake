# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_repository(name):
    github_archive(
        name = name,
        repository = "syoyo/tinyobjloader",
        commit = "v1.0.6",
        sha256 = "19ee82cd201761954dd833de551edb570e33b320d6027e0d91455faf7cd4c341",  # noqa
        build_file = "@drake//tools/workspace/tinyobjloader:package.BUILD.bazel",  # noqa
    )
