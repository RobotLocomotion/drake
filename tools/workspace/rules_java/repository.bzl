load("//tools/workspace:github.bzl", "github_archive")
load("//tools/workspace:workspace_deprecation.bzl", "print_warning")

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def rules_java_repository(
        name,
        mirrors = None,
        _is_drake_self_call = False):
    if not _is_drake_self_call:
        print_warning("rules_java_repository")
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
        commit = "8.11.0",
        sha256 = "06e6e30859c5fc49ac1e6e7fdaffc0df110049debbd429686ad65bb5b425f7b5",  # noqa
        mirrors = mirrors,
    )
