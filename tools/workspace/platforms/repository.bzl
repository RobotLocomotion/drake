load("//tools/workspace:github.bzl", "github_archive")

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def platforms_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/platforms",  # License: Apache-2.0
        upgrade_advice = """
        When updating, you must also manually propagate to the new version
        number into the MODULE.bazel file (at the top level of Drake).
        """,
        commit = "0.0.10",
        sha256 = "3df33228654e56b09f17613613767b052581b822d57cb9cfd5e7b19a8e617b42",  # noqa
        mirrors = mirrors,
    )
