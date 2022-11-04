# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def uwebsockets_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "uNetworking/uWebSockets",
        # TODO(jwnimmer-tri) Switch back to numbered releases as of > v20.29.0.
        commit = "654b4558a4347cc9f437f82c6000639af5f20c2c",
        sha256 = "d0a89a019ed1ee7515c245bb8e3593cead92b499d431c9ca813cb9647d6e84c7",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
