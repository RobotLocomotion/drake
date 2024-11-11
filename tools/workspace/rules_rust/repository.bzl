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
        commit = "0.53.0",
        sha256 = "f8a76f6fae5ffb9f309f87333f6f0f47c1902b7891b5bd59001988eb0d5cb291",  # noqa
        patches = [
            ":patches/import_cycle.patch",
        ] + (extra_patches or []),
        mirrors = mirrors,
    )
