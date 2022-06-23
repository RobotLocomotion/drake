# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def u_msgpack_python_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "vsergeev/u-msgpack-python",
        commit = "v2.7.1",
        sha256 = "3cfb5a8fb0d784522c88cea2473b6f879f004118d23cdef29660d34a983d7c87",  # noqa
        build_file = "@drake//tools/workspace/u_msgpack_python_internal:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
