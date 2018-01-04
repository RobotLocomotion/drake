# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def ignition_rndf_repository(name):
    bitbucket_archive(
        name = name,
        repository = "ignitionrobotics/ign-rndf",
        commit = "0e7c72e6f5f4",
        sha256 = "a90b8e2e53e284df7e2876e999c4232a03a260c5b6c0a553016ba0a9fab934ad",  # noqa
        strip_prefix = "ignitionrobotics-ign-rndf-0e7c72e6f5f4",
        build_file = "@drake//tools/workspace/ignition_rndf:package.BUILD.bazel",  # noqa
    )
