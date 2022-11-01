# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def usockets_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "uNetworking/uSockets",
        commit = "v0.8.2",
        sha256 = "c17fc99773a30552cdd7741ff98af177eeecb26b89fc38011b430956b3b2eca5",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
