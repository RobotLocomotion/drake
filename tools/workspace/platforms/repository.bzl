load("//tools/workspace:github.bzl", "github_archive")
load("//tools/workspace:workspace_deprecation.bzl", "print_warning")

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def platforms_repository(
        name,
        mirrors = None,
        _is_drake_self_call = False):
    if not _is_drake_self_call:
        print_warning("platforms_repository")
    github_archive(
        name = name,
        repository = "bazelbuild/platforms",  # License: Apache-2.0
        upgrade_advice = """
        When updating, you must also manually propagate to the new version
        number into the MODULE.bazel file (at the top level of Drake).
        """,
        commit = "0.0.11",
        sha256 = "ace459f5d033c86e6578df817f739b21101c0ebcd409a97badc2979c22ce9fdc",  # noqa
        mirrors = mirrors,
    )
