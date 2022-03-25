# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ignitionrobotics/sdformat",
        commit = "sdf12",
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        sha256 = "af34d335c19b3dab62708d988e28d19bb041cff881c66627b8816a14be8b722f",  # noqa
        patches = [
            "@drake//tools/workspace/sdformat:patches/console.patch",
            "@drake//tools/workspace/sdformat:patches/no_urdf.patch",
        ],
        mirrors = mirrors,
    )
