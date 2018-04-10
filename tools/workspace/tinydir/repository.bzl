# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def tinydir_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "cxong/tinydir",
        commit = "677733daa2859c963da953872f8d591251c2ae5e",
        sha256 = "ac87282bf2a127df61fabe2eb2e4cbe2adb2050ecf3c4b9885ffddd4bf887125",  # noqa
        build_file = "@drake//tools/workspace/tinydir:package.BUILD.bazel",
        mirrors = mirrors,
    )
