# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def sdformat_repository(
        name,
        mirrors = None):
    # From the following branch:
    # https://bitbucket.org/osrf/sdformat/commits/branch/default
    commit = "21d2cbe52bbd"  # v6.2.0
    bitbucket_archive(
        name = name,
        repository = "osrf/sdformat",
        commit = commit,
        sha256 = "fe70ae9c79ecc043b492bc7170be96bb02bf1707820f243a8671f5be12754da8",  # noqa
        strip_prefix = "osrf-sdformat-%s" % (commit),
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
    )
