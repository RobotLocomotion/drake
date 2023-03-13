load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "61c02fb47fa90ac1a5daa83e22d8f39bb1348aaf",
        sha256 = "58a8485ec1e6001f3987f5a6d7e3f8ee38af898aa5f93a07e1c4f5025ef42de7",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/sre_deprecation.patch",
            ":patches/test_paths.patch",
        ],
        mirrors = mirrors,
    )
