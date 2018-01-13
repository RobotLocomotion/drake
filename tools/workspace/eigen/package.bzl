# -*- python -*-

load("@drake//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def eigen_repository(name):
    bitbucket_archive(
        name = name,
        repository = "eigen/eigen",
        # N.B. See #5785; do your best not to bump this to a newer commit.
        commit = "3.3.3",
        sha256 = "94878cbfa27b0d0fbc64c00d4aafa137f678d5315ae62ba4aecddbd4269ae75f",  # noqa
        strip_prefix = "eigen-eigen-67e894c6cd8f",
        build_file = "@drake//tools/workspace/eigen:package.BUILD.bazel",
    )
