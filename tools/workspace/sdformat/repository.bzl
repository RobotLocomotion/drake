# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "azeey/sdformat",
        commit = "a7f9108fd36d8a20e56bcc12be7bdc46e4fd3ca9",
        sha256 = "b2f9926f6f4f54e813d438f12ef0ebc50ef60df984ef9bf7b04724053f62ea1a",  # noqa
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
    )
