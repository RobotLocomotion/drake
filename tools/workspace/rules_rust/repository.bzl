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
        commit = "0.41.1",
        sha256 = "018bd3c70aec1103ce8ac18c98765672f07e6b99218d67001c27b53167e6eb01",  # noqa
        patches = [
            ":patches/import_cycle.patch",
        ] + (extra_patches or []),
        mirrors = mirrors,
    )
