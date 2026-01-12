load("//tools/workspace:github.bzl", "github_archive")

# Everything here is deprecated for removal on 2026-02-01.

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        # TODO(jwnimmer-tri) At the moment we have both @lcm and @lcm_internal
        # repositories, with @lcm being deprecated. Be aware that we have
        # different pins for each repository.
        commit = "v1.5.1",
        sha256 = "40ba0b7fb7c9ad06d05e06b4787d743cf11be30eb4f1a03abf4a92641c5b1203",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
