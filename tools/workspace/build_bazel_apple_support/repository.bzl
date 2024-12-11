load("//tools/workspace:github.bzl", "github_archive")

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def build_bazel_apple_support_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/apple_support",  # License: Apache-2.0
        upgrade_advice = """
        When updating, you must also manually propagate to the new version
        number into the MODULE.bazel file (at the top level of Drake).
        """,
        commit = "1.17.1",
        sha256 = "cfc295c5acb751fc3299425a1852e421ec0a560cdd97b6a7426d35a1271c2df5",  # noqa
        patches = [
            ":patches/no_bazel_features.patch",
        ],
        mirrors = mirrors,
    )
