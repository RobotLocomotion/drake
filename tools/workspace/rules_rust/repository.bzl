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
        upgrade_advice = """
        An upgrade to @rules_rust also requires re-pinning the toolchain.
        Run `drake/tools/workspace/rust_toolchain/upgrade.sh`.
        """,
        commit = "0.28.0",
        sha256 = "e2f32fad3539bd57527901981ef6fb200d71030c30994bdf48faa6f56683f2d0",  # noqa
        patches = extra_patches,
        mirrors = mirrors,
    )
