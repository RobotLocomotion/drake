# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def nlopt_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "stevengj/nlopt",
        commit = "v2.7.1",
        sha256 = "db88232fa5cef0ff6e39943fc63ab6074208831dc0031cf1545f6ecd31ae2a1a",  # noqa
        build_file = "@drake//tools/workspace/nlopt_internal:package.BUILD.bazel",  # noqa
        patches = [
            "@drake//tools/workspace/nlopt_internal:patches/remove_luksan.patch",  # noqa
            "@drake//tools/workspace/nlopt_internal:patches/vendoring.patch",
        ],
        mirrors = mirrors,
    )
