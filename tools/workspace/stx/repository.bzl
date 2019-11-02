# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def stx_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "tcbrindle/cpp17_headers",
        commit = "6d8157a06159e49b90a23c6405598de4a017a86c",
        sha256 = "6d9752c7f5ba4d31ac8c859d3787b04ffcb8321b874db8497e76d973145e2768",  # noqa
        build_file = "@drake//tools/workspace/stx:package.BUILD.bazel",
        mirrors = mirrors,
    )
