load("@drake//tools/workspace:github.bzl", "github_archive")

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def rules_rust_tinyjson_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "rhysd/tinyjson",  # License: Apache-2.0
        commit = "v2.5.1",
        sha256 = "4427e79284ef1b445d338a47bb66f7d79dda60dce05ca3eac5f2daecb51138e2",  # noqa
        build_file = "@rules_rust//util/process_wrapper:BUILD.tinyjson.bazel",
        mirrors = mirrors,
    )
