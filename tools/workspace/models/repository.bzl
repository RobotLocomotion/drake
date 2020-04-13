# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "f8ad89d34325fb408ae0d9f4d395db5820a4959d",
        sha256 = "2241e2f3af0f4a20557794ea36da89c5a852a8d0f6e6c93e0fb56c6811fdabf4",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
