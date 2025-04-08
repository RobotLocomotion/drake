load("//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "0f60e8581d44ec9e619e18fac2d5643cc9284b44",
        sha256 = "9af06bcd0062232c2f32f5e6b89cefb726a8605498c1e20f7b18a6886a152634",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/sre_deprecation.patch",
            ":patches/test_paths.patch",
        ],
        mirrors = mirrors,
    )
