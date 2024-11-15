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
        commit = "0.0.16",
        sha256 = "bbf1ae2f83305b7053b11e4467d317a7ba3517a12cef608543c1b1c5bf48a4df",  # noqa
        patches = [
            ":patches/rm_protobuf.patch",
        ],
        mirrors = mirrors,
    )
