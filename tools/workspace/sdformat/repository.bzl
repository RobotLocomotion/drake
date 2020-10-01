# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "osrf/sdformat",
        commit = "sdformat9_9.3.0",
        sha256 = "6886cd905c74698000bf4e4bb378efe292411fab939d80d3263dfad430e50204",  # noqa
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        patches = [
            # TODO(jwnimmer-tri) This patch is cherry-picked from upstream; we
            # should remove it once we reach a new enough version, probably for
            # sdformat10 or so.
            "@drake//tools/workspace/sdformat:3bbd303.patch",
            # Disable logging to $HOME/.sdformat/sdformat.log
            # TODO(jwnimmer-tri) Once osrf/sdformat#338 is released, we can
            # remove this patch and set SDFORMAT_DISABLE_CONSOLE_LOGFILE=1
            # instead, probably for sdformat10 or so.
            "@drake//tools/workspace/sdformat:no-console-file.patch",
        ],
        patch_args = ["-p1"],
        mirrors = mirrors,
    )
