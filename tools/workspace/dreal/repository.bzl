# -*- mode: python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def dreal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "dreal/dreal4",
        # TODO(jwnimmer-tri) Prefer to use a numbered release here, once there
        # is a newer one than 4.19.10.2.
        commit = "61a199ee9d99fb9913ed51c9569e7f0c36ce1476",
        sha256 = "c0230df36fab72b8c28f5139b6fde5df2094e9c30cf9ad8065bade47903732e1",  # noqa
        mirrors = mirrors,
    )
