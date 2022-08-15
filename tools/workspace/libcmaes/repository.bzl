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
        commit = "17cbf58aec13c1d494fa0fed826f78560b817de1",
        sha256 = "759ab2d70d6d86d7d8fce6d80863a6368d7d245c96116c0f70cbc7c144873d51",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
