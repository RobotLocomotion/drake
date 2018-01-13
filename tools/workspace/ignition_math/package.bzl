# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def ignition_math_repository(name):
    bitbucket_archive(
        name = name,
        repository = "ignitionrobotics/ign-math",
        commit = "392237e10ba4",
        sha256 = "44068bb91c07c9305213057cad801ae5b689ac1a5f37cd8330dd6e729df8f5b0",  # noqa
        strip_prefix = "ignitionrobotics-ign-math-392237e10ba4",
        build_file = "@drake//tools/workspace/ignition_math:package.BUILD.bazel",  # noqa
    )
