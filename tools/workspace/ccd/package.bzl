# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def ccd_repository(name):
    github_archive(
        name = name,
        repository = "danfis/libccd",
        commit = "5677d384315d64c41a9e1dabe6a531f10ffbb7fb",
        sha256 = "3b37ef4555d087f7abb6aa59c3b5cecb96410ea10e95a086ef2771569fb6fdfb",  # noqa
        build_file = "@drake//tools/workspace/ccd:package.BUILD.bazel",
    )
