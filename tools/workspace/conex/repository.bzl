# -*- python -*-
load("@drake//tools/workspace:github.bzl", "github_archive")

def conex_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ToyotaResearchInstitute/conex",
        commit = "e53ebc612ccb5c4da757bbe90f7be07ff13db0a4",
        sha256 = "adfaef47709a751da0c8445719fb97f133acf79c3e75594b2663e89be76879e4",  # noqa
        build_file = "@drake//tools/workspace/conex:package.BUILD.bazel",
        mirrors = mirrors,
    )
