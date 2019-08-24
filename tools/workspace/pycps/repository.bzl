# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pycps_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "mwoehlke/pycps",
        commit = "793363729e60bfdc4b0b1d7a99038aa481aac960",
        sha256 = "1c304023e5dbce21a423cd5665ee0e3f2deadcd42413bb1bca36d2bd747f1564",  # noqa
        build_file = "@drake//tools/workspace/pycps:package.BUILD.bazel",
        mirrors = mirrors,
    )
