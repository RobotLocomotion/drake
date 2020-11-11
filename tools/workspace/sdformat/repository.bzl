# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # repository = "azeey/sdformat",
        # commit = "732be0f20f00cfb1f5200f691c33abee3eee7a02",
        # sha256 = "4d537b3cbebdefcea57039aea487ae8603058e581efac0402a5a69757194738c",  # noqa
        repository = "osrf/sdformat",
        commit = "sdformat10_10.0.0",
        sha256 = "3555ce443a736e9bd24577d894978e0236212bfd2b356022fd53b143b30152bf",  # noqa
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
    )
