# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def stx_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "tcbrindle/cpp17_headers",
        commit = "4f82af87fd97969a89ed0018ae72e5dacc8d308a",
        sha256 = "7544aa3af121be49672e7dd227a4753bf063a10abda5c4cab9c094d423b6635e",  # noqa
        build_file = "@drake//tools/workspace/stx:package.BUILD.bazel",
        mirrors = mirrors,
    )
