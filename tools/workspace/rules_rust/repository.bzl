load("@drake//tools/workspace:github.bzl", "github_archive")

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def rules_rust_repository(
        name,
        mirrors = None,
        extra_patches = None):
    github_archive(
        name = name,
        repository = "bazelbuild/rules_rust",  # License: Apache-2.0
        commit = "0.38.0",
        sha256 = "5873326d431bdd5fc0a3e0be3d060580ea0ea477c62bd64e299be1b0b7eeacf4",  # noqa
        patches = (extra_patches or []) + [
            ":patches/license_filegroup.patch",
        ],
        mirrors = mirrors,
    )
