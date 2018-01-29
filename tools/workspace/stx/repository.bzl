# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def stx_repository(name):
    github_archive(
        name = name,
        repository = "tcbrindle/cpp17_headers",
        commit = "5f4e44153d42335c68027ec8e2f84db070837a88",
        sha256 = "c42239a5ce1e74d8464b5a86d7b9257ab6576c2a1b06afa80e69a97c00f7f525",  # noqa
        build_file = "@drake//tools/workspace/stx:package.BUILD.bazel",
    )
