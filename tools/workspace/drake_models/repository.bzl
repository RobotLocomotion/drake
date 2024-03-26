load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "69c92595a391eb023c27ab6ac8f80d58a3e4612d",
        sha256 = "ba571c8b369a62c3764d250944b27d72071488789b2494604d23342994141fe2",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
