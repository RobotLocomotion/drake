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
        commit = "0.30.0",
        sha256 = "984def6785d76487c27087048b59639900f8e0ddba4fc9c265eb9134c50c0b50",  # noqa
        patches = (extra_patches or []) + [
            ":patches/license_filegroup.patch",
        ],
        mirrors = mirrors,
    )
