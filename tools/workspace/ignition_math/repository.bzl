# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def ignition_math_repository(
        name,
        mirrors = None):
    # When updating this commit, also remember to adjust the PROJECT_*
    # constants in ./package.BUILD.bazel to match the new version number.
    commit = "99593fcb7c24"
    bitbucket_archive(
        name = name,
        repository = "ignitionrobotics/ign-math",
        commit = commit,
        sha256 = "e6ec643f733e8cdbf358c763f29a436c0e6084e2479be57e5f67bc9988ba77d3",  # noqa
        strip_prefix = "ignitionrobotics-ign-math-%s" % (commit),
        build_file = "@drake//tools/workspace/ignition_math:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
