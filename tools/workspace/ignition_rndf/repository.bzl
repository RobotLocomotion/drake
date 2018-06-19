# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def ignition_rndf_repository(
        name,
        mirrors = None):
    # When updating this commit, also remember to adjust the PROJECT_*
    # constants in ./package.BUILD.bazel to match the new version number.
    commit = "0e7c72e6f5f4"
    bitbucket_archive(
        name = name,
        repository = "ignitionrobotics/ign-rndf",
        commit = commit,
        sha256 = "a90b8e2e53e284df7e2876e999c4232a03a260c5b6c0a553016ba0a9fab934ad",  # noqa
        strip_prefix = "ignitionrobotics-ign-rndf-%s" % (commit),
        build_file = "@drake//tools/workspace/ignition_rndf:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
