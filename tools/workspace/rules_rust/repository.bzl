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
        When updating, you must also manually propagate to the new version
        number into the MODULE.bazel file (at the top level of Drake).
        """,
        commit = "0.55.6",
        sha256 = "f1d4ba0893277e766574e8be3bf9f414436215d5779ef5f14a7180d5b01ea264",  # noqa
        patches = [
            ":patches/import_cycle.patch",
        ] + (extra_patches or []),
        mirrors = mirrors,
    )
