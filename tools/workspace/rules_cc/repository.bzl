load("//tools/workspace:github.bzl", "github_archive")

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def rules_cc_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/rules_cc",  # License: Apache-2.0,
        commit = "0.0.17",
        # When using WORKSPACE, the >= 0.1.x series of rules_cc causes trouble.
        commit_pin = True,
        sha256 = "abc605dd850f813bb37004b77db20106a19311a96b2da1c92b789da529d28fe1",  # noqa
        patches = [
            ":patches/rm_protobuf.patch",
        ],
        mirrors = mirrors,
    )
