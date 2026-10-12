load("//tools/workspace:github.bzl", "github_archive")

def uwebsockets_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "uNetworking/uWebSockets",
        upgrade_type = "release",
        commit = "v20.80.0",
        sha256 = "561d382837f4b78da7e4fccb218f037f6fc0b4859fceff00fe7ce052e0bcb218",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/max_fallback_size.patch",
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
