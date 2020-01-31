# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def sdformat_repository(
        name,
        mirrors = None):
    # From tags listed here:
    # https://bitbucket.org/osrf/sdformat/downloads/?tab=tags
    commit = "632993e4b142"  # tag/sdformat9_9.1.0
    bitbucket_archive(
        name = name,
        repository = "osrf/sdformat",
        commit = commit,
        sha256 = "407599011850143d4de8ca9d5d7370b9f48f2b1d6404d381dfdd4ec23b4e6520",  # noqa
        strip_prefix = "osrf-sdformat-%s" % (commit),
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
    )
