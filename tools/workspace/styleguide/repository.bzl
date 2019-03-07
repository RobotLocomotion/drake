# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "51c4a55a09abeb17c5f43f521f348452ffd509dd",
        sha256 = "506a5f36581d6a420414d6fe6edc048dade4134f93de53e94b8400afbd7f152d",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
