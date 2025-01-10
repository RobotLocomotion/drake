load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "43a7e91dde0f8193ccbbdc6a0e49faf7a1aaeefc",
        sha256 = "fc4a2817d9f26ef119da8d5ae994af472a5a13edddd62a75bb5373fb4bf3a4cc",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
