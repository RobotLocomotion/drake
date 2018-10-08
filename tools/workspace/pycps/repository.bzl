# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pycps_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # PR DRAFT: Update this once upstream PR lands.
        repository = "mwoehlke/pycps",
        commit = "cd77b4cec588253ea22bb2a262e0cc6b9400d5e7",
        sha256 = "d3c6002ccc11b6cb01626098a11c91ceca719920e39bac6bc0eeece225bf67cc",  # noqa
        build_file = "@drake//tools/workspace/pycps:package.BUILD.bazel",
        mirrors = mirrors,
    )
