# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def sdformat_repository(
        name,
        mirrors = None):
    # From the following branch:
    # https://bitbucket.org/osrf/sdformat/commits/branch/default
    commit = "1a3f95acdc3c"  # tag/sdformat9_9.0.0
    bitbucket_archive(
        name = name,
        repository = "osrf/sdformat",
        commit = commit,
        sha256 = "a0b4d809c5ebbaaf9f5eb344ba779fa0714047ed5676a4ccc63808fc6db94fcc",  # noqa
        strip_prefix = "osrf-sdformat-%s" % (commit),
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
        # TODO(eric.cousineau): Remove this once libsdformat incorporates it.
        patches = ["@drake//tools/workspace/sdformat:Converter.cc.diff"],
    )
