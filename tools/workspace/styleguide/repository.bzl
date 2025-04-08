load("//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "4936ea9c91658a2d9f2e0329cb19c748b28dde6e",
        sha256 = "d4c03fd726af6a1a33c8fbf918b38d3062df90ea20ac70c597ad7423ba53021b",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/sre_deprecation.patch",
            ":patches/test_paths.patch",
        ],
        mirrors = mirrors,
    )
