# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def sdformat_repository(
        name,
        mirrors = None):
    # From the following branch:
    # https://bitbucket.org/osrf/sdformat/commits/branch/default
    commit = "e76f526e7e2e"  # vx.y.z DNM non-default!
    bitbucket_archive(
        name = name,
        repository = "osrf/sdformat",
        commit = commit,
        sha256 = "826391f4ddb7b0b6f07e62d9d7b5609834ee1d8ab3c514c0d66187519bd6631c",  # noqa
        strip_prefix = "osrf-sdformat-%s" % (commit),
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
    )
