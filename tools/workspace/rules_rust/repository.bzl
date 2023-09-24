load("@drake//tools/workspace:github.bzl", "github_archive")

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def rules_rust_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/rules_rust",  # License: Apache-2.0
        commit = "0.27.0",
        sha256 = "d9a3981f4ef18ced850341bc05c7e2a506006a47a0207b6f7191f271cb893233",  # noqa
        mirrors = mirrors,
    )
