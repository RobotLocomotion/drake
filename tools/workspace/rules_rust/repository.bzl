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
        Run `drake/tools/workspace/rust_toolchain/upgrade.py`.
        """,
        commit = "0.36.2",
        sha256 = "dbc6e2a8fe8bef29adb2d0ceb612cb136ca93d47d3a77355eea9077a28dc7d32",  # noqa
        patches = (extra_patches or []) + [
            ":patches/license_filegroup.patch",
        ],
        mirrors = mirrors,
    )
