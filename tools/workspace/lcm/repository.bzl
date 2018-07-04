# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        commit = "8f7189bd1db9dfeb66cd4ed0bfd34edba819f46e",
        sha256 = "be4559347675dd1c9729aa5e766874ec836ba89e255326e52f943e1de6e2f4dd",  # noqa
        build_file = "@drake//tools/workspace/lcm:package.BUILD.bazel",
        mirrors = mirrors,
    )
