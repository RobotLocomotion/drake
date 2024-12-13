load("//tools/workspace:github.bzl", "github_archive")

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def rules_java_repository(
        name,
        mirrors = None):
    if native.bazel_version[0:2] == "7.":
        # The new rules_java only works with Bazel 8; for bazel 7 we'll use the
        # built-in rules_java. We can remove this once Drake's minimum Bazel
        # version is >= 8.
        return
    github_archive(
        name = name,
        repository = "bazelbuild/rules_java",  # License: Apache-2.0,
        upgrade_advice = """
        When updating, you must also manually propagate to the new version
        number into the MODULE.bazel file (at the top level of Drake).
        """,
        commit = "8.6.1",
        sha256 = "b2519fabcd360529071ade8732f208b3755489ed7668b118f8f90985c0e51324",  # noqa
        mirrors = mirrors,
    )
