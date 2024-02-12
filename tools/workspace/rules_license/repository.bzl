load("//tools/workspace:github.bzl", "github_archive")

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def rules_license_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/rules_license",  # License: Apache-2.0,
        commit = "0.0.8",
        sha256 = "8c1155797cb5f5697ea8c6eac6c154cf51aa020e368813d9d9b949558c84f2da",  # noqa
        mirrors = mirrors,
    )
