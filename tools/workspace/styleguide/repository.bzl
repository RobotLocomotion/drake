load("//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "736a797e4bc25a2c064fee147e659004675b7387",
        sha256 = "6fb793292a94baf75b9ad1fb91910cf710b5b1a36823e09abf17ffbea8b06aec",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/sre_deprecation.patch",
            ":patches/test_paths.patch",
        ],
        mirrors = mirrors,
    )
