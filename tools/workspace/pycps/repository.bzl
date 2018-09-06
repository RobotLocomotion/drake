# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pycps_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "mwoehlke/pycps",
        commit = "c47b5535e02af0f3d367fcd49510ac7fbc374ba9",
        sha256 = "c5aa5c3261aa2312572ae8af2730b904f49e6571517ead60017d65a27aa628aa",  # noqa
        build_file = "@drake//tools/workspace/pycps:package.BUILD.bazel",
        mirrors = mirrors,
    )
