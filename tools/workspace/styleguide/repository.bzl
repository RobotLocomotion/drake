load("//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "b914285784092231fab393c2b8d79f8e3495b297",
        sha256 = "d496a290e0d189c60af142211911f7dbee0c9fe1e835b219ce7b522c9f0e7c31",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/sre_deprecation.patch",
            ":patches/test_paths.patch",
        ],
        mirrors = mirrors,
    )
