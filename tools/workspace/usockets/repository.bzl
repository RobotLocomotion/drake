# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def usockets_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "uNetworking/uSockets",
        # TODO(jwnimmer-tri) Switch back to numbered releases as of > v0.8.2.
        # https://github.com/uNetworking/uSockets/issues/188
        commit = "a0490a84c6555477c2af301cd3b90e326427685f",
        sha256 = "0370d9ae2576221b37bb315c89008e9ba81f23ddcb0d6425763a98aa599544c7",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
