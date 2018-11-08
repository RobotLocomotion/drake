# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pycps_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "mwoehlke/pycps",
        commit = "83326ffaf03528e83f673eccb8af9b129504bf42",
        sha256 = "eede05c985d35f683445d8267be1ad7679b296b2d00290870c347e91822f51d1",  # noqa
        build_file = "@drake//tools/workspace/pycps:package.BUILD.bazel",
        mirrors = mirrors,
    )
