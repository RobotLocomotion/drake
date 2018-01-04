# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def ignition_math_repository(name):
    bitbucket_archive(
        name = name,
        repository = "ignitionrobotics/ign-math",
        commit = "568cf1457760",
        sha256 = "bd0cafb01cc219c5e12c6b049cae44cfa68bb39bdb52a3c79c37f2163d7d4967",  # noqa
        strip_prefix = "ignitionrobotics-ign-math-568cf1457760",
        build_file = "@drake//tools/workspace/ignition_math:package.BUILD.bazel",  # noqa
    )
