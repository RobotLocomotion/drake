# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_repository(
        name,
        local_repository_override = "/home/aaron/workspaces/sdf12/src/sdformat",
        mirrors = None):
    github_archive(
        name = name,
        repository = "ignitionrobotics/sdformat",
        commit = "aaron/read-file-with-config",
        sha256 = "5daaa86bb4e6015041f555f5c78e8d5ce1bcbc835f1df945c1ec15ef932c9a94",
        # commit = "sdformat12_12.0.0",
        # sha256 = "b3f44b7bc4530ca40ca120489d791d8ee9295beb30a16406926fb2ae42b3fba8",  # noqa
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
    )
