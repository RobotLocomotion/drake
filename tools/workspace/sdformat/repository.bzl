# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "osrf/sdformat",
        commit = "sdformat9_9.1.0",
        sha256 = "e5ac1e7b9b37edf2236a87c7ef4c82d20710439d16933a7e4602a31c82e95367",  # noqa
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        patches = [
            # TODO(jwnimmer-tri) This patch is cherry-picked from upstream; we
            # should remove it once we reach a new enough version, probably for
            # sdformat10 or so.
            "@drake//tools/workspace/sdformat:3bbd303.patch",
        ],
        mirrors = mirrors,
    )
