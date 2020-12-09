# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def common_robotics_utilities_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ToyotaResearchInstitute/common_robotics_utilities",
        commit = "a2f167220ac5b0cc7b7bb7b1383d1f8c77ca22b1",
        sha256 = "67d6a6bf3ea18d29cf619e5a28b0e536a263b420c6dd5561c3d31974f170a9fa",  # noqa
        build_file = "//tools/workspace/common_robotics_utilities:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
