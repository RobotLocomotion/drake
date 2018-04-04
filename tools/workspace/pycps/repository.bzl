# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pycps_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "mwoehlke/pycps",
        commit = "fc2ef939929131a32d857d0728a11ecf08b28caa",
        sha256 = "088eb74bab3498c6a8c4ff3f951fc777dc1c32123c6ce1e14f774bfbb32afd37",  # noqa
        build_file = "@drake//tools/workspace/pycps:package.BUILD.bazel",
        mirrors = mirrors,
    )
