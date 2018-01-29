# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def tinydir_repository(name):
    github_archive(
        name = name,
        repository = "cxong/tinydir",
        commit = "3aae9224376b5e1a23fd824f19d9501162620b53",
        sha256 = "fa7eec0baaa5f6c57df1a38b064ec3a4f098f477f0a64d97c646a4470ffdd3b6",  # noqa
        build_file = "@drake//tools/workspace/tinydir:package.BUILD.bazel",
    )
