# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def ignition_math_repository(
        name,
        mirrors = None):
    # When updating this commit, also remember to adjust the PROJECT_*
    # constants in ./package.BUILD.bazel to match the new version number.
    commit = "11bba67b3176"  # tag/ignition-math6_6.4.0
    bitbucket_archive(
        name = name,
        repository = "ignitionrobotics/ign-math",
        commit = commit,
        sha256 = "5b9835f07771946dbc051c4677928bf578a874464d8ada1d6b4d6fa5881e5427",  # noqa
        strip_prefix = "ignitionrobotics-ign-math-%s" % (commit),
        build_file = "@drake//tools/workspace/ignition_math:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
