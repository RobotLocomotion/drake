load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "5c942636d18013870403c17c8209558799122abd",
        sha256 = "ac0d6a943818a9ac07d24224ecfbadd1632946de066954901c1aa4c16319fb38",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
