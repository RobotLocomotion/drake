# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def sdformat_repository(
        name,
        mirrors = None):
    commit = "13546ead5493"
    bitbucket_archive(
        name = name,
        repository = "osrf/sdformat",
        commit = commit,
        sha256 = "87a61f3cdef4b96fbe0bed1ab0e462c3d40f745251f8312bc10b55cfd3450d80",  # noqa
        strip_prefix = "osrf-sdformat-%s" % (commit),
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
    )
