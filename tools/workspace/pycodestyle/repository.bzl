# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pycodestyle_repository(name):
    github_archive(
        name = name,
        repository = "PyCQA/pycodestyle",
        commit = "2.3.1",
        sha256 = "e9fc1ca3fd85648f45c0d2e33591b608a17d8b9b78e22c5f898e831351bacb03",  # noqa
        build_file = "@drake//tools/workspace/pycodestyle:package.BUILD.bazel",  # noqa
    )
