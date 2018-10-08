# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pycps_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # PR DRAFT: Update this once upstream PR lands.
        repository = "EricCousineau-TRI/pycps",
        commit = "2e204d055cf8d9fd79554dc61562c41ac169f11d",
        sha256 = "8951f3150da599ae0074788e06eb1a8fb6d2f8a6c8e29e9b1ce1285f10b8c56e",  # noqa
        build_file = "@drake//tools/workspace/pycps:package.BUILD.bazel",
        mirrors = mirrors,
    )
