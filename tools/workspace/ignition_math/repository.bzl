# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def ignition_math_repository(
        name,
        mirrors = None):
    commit = "abac877ac1e3"
    bitbucket_archive(
        name = name,
        repository = "ignitionrobotics/ign-math",
        commit = commit,
        sha256 = "37237e375cbd36c1aa1e2c72c97b16f9be95d2b7562bffda1842e2016d4a4794",  # noqa
        strip_prefix = "ignitionrobotics-ign-math-%s" % (commit),
        build_file = "@drake//tools/workspace/ignition_math:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
