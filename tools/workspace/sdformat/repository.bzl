# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "azeey/sdformat",
        commit = "compute_proxy_name",
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        sha256 = "3a5eb9c537c1ef022491e949f3626a6ec015e306a88d2931b384995aa9c2d760",  # noqa
        patches = [
            "@drake//tools/workspace/sdformat:patches/console.patch",
        ],
        mirrors = mirrors,
    )
