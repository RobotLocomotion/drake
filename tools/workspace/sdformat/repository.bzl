# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def sdformat_repository(
        name,
        mirrors = None):
    # From the following branch:
    # https://bitbucket.org/osrf/sdformat/commits/branch/default
    commit = "c012aa7de48c"  # vx.y.z DNM non-default!
    bitbucket_archive(
        name = name,
        repository = "osrf/sdformat",
        commit = commit,
        sha256 = "6bc3e09ba91920a6e37fe616b2cf455e1a62b8dcc5cd3ad4b07ae08757d4609c",  # noqa
        strip_prefix = "osrf-sdformat-%s" % (commit),
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
    )
