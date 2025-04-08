load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "af1d93cd273f07bf3d969bb37e93e190036d1a0b",
        sha256 = "1b275bfa26476baba28a0d4dc8e36db17641bd832cefaf0ee83ed285249e2753",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
