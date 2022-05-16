# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def libcmaes_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "CMA-ES/libcmaes",
        # TODO(jwnimmer-tri) We use an untagged commit, in order to use the
        # Apache-2.0 license. Any time we upgrade this to a newer commit, we
        # should check if there is an official version number yet that we
        # could use (i.e., newer than v0.10).
        commit = "1c39d7f931b267117d365370c6da5334a6948942",
        sha256 = "0abb557242da5a4b785e11aec1c00e57d583f23f04579921da9110206d0a6767",  # noqa
        build_file = "@drake//tools/workspace/libcmaes:package.BUILD.bazel",
        mirrors = mirrors,
    )
