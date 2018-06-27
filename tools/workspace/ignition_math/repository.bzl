# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def ignition_math_repository(
        name,
        mirrors = None):
    # When updating this commit, also remember to adjust the PROJECT_*
    # constants in ./package.BUILD.bazel to match the new version number.
    commit = "cd8be05d47f5"
    bitbucket_archive(
        name = name,
        repository = "ignitionrobotics/ign-math",
        commit = commit,
        sha256 = "2d3832283c8588e03bd5853f9eb4e4319ab57c2d04fd229d69d7411a6abe83ac",  # noqa
        strip_prefix = "ignitionrobotics-ign-math-%s" % (commit),
        build_file = "@drake//tools/workspace/ignition_math:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
