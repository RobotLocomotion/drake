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
        commit = "0.32.0",
        sha256 = "2d42ff62c076674bfcaea3d6dea76004db36f118f1773002fd9f23538babccd9",  # noqa
        patches = (extra_patches or []) + [
            ":patches/license_filegroup.patch",
        ],
        mirrors = mirrors,
    )
