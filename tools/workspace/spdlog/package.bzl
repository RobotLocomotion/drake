# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def spdlog_repository(name):
    github_archive(
        name = name,
        repository = "gabime/spdlog",
        commit = "v0.13.0",
        sha256 = "d798a6ca19165f0a18a43938859359269f5a07fd8e0eb83ab8674739c9e8f361",  # noqa
        build_file = "@drake//tools/workspace/spdlog:package.BUILD.bazel",
    )
