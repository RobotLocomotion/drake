# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:github.bzl", "github_archive")

def msgpack_lite_js_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "kawanet/msgpack-lite",
        commit = "5b71d82cad4b96289a466a6403d2faaa3e254167",
        sha256 = "e0a3f03a85fe7748257747b289a6062efdc2cfa5f08cefc3dbdebe45eae0904b",  # noqa
        build_file = "@drake//tools/workspace/msgpack_lite_js:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
