load("//tools/workspace:github.bzl", "github_archive")
load("//tools/workspace:workspace_deprecation.bzl", "print_warning")

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def rules_cc_repository(
        name,
        mirrors = None,
        _is_drake_self_call = False):
    if not _is_drake_self_call:
        print_warning("rules_cc_repository")
    github_archive(
        name = name,
        repository = "bazelbuild/rules_cc",  # License: Apache-2.0,
        commit = "0.1.1",
        sha256 = "712d77868b3152dd618c4d64faaddefcc5965f90f5de6e6dd1d5ddcd0be82d42",  # noqa
        patches = [
            ":patches/rm_protobuf.patch",
        ],
        mirrors = mirrors,
    )
