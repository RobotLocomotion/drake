# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "osrf/sdformat",
        commit = "sdformat9_9.2.0",
        sha256 = "0e42001d92aa2c089c7d0c4ea6a30db2beeff0af3a9a357e7ccd0a4e1131cae7",  # noqa
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        patches = [
            # TODO(jwnimmer-tri) This patch is cherry-picked from upstream; we
            # should remove it once we reach a new enough version, probably for
            # sdformat10 or so.
            "@drake//tools/workspace/sdformat:3bbd303.patch",
        ],
        patch_args = ["-p1"],
        mirrors = mirrors,
    )
