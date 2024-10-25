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
        commit = "0.52.0",
        sha256 = "8d44ac4e33308f617106abe50a4c2d2b415c0a6216a6de41f1b863432263d60a",  # noqa
        patches = [
            ":patches/import_cycle.patch",
        ] + (extra_patches or []),
        mirrors = mirrors,
    )
