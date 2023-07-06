load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "6aae49e2b5861cd298d681d628830a6261b947c3",
        sha256 = "4a46dedf35d61c0dfeeb4e3ce6c6593c59265624332d849268a787ca1cc95a2a",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/sre_deprecation.patch",
            ":patches/test_paths.patch",
        ],
        mirrors = mirrors,
    )
