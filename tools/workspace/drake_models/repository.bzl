load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "6b67e4c4b915bb42965782b3ddb803d48daf2a52",
        sha256 = "aa3a2472fa579cc8362a6c83b27db1bcac4956c7565a28491b3652e721a55996",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
