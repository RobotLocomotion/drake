# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def uritemplate_py_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python-hyper/uritemplate",
        commit = "4.1.1",
        sha256 = "64cae94edd83bbb0c2c49b15f2cb8192c3f8492af6bc468211d1e8b8496f5791",  # noqa
        build_file = "@drake//tools/workspace/uritemplate_py:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
