# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def spdlog_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gabime/spdlog",
        # This the commit immediately following v0.16.3 that fixes the version
        # badging.  We should change back to saying "v0.17.1" or whatever here
        # (instead of a commit hash) upon the next upstream release.
        commit = "f258af4364ed2aa966ddce8292b9bbde8bbb6152",
        sha256 = "69799a0963fe396e569bedcc9263511a61e3b6dc586bd800bd9597ad3c2268f0",  # noqa
        build_file = "@drake//tools/workspace/spdlog:package.BUILD.bazel",
        mirrors = mirrors,
    )
