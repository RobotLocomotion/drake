load("//tools/workspace:github.bzl", "github_archive")
load("//tools/workspace/buildifier:buildifier.bzl", "buildifier_repository")

ALL = [
    "pycodestyle",
    "buildifier",
]

def lint_repositories(include = ALL):
    if "pycodestyle" in include:
        github_archive(
            name = "pycodestyle",
            repository = "PyCQA/pycodestyle",
            commit = "2.3.1",
            sha256 = "e9fc1ca3fd85648f45c0d2e33591b608a17d8b9b78e22c5f898e831351bacb03",  # noqa
            # `build_file` must be specified as a target `:` so that Bazel
            # knows how to resolve it thorugh an external.
            build_file = "@drake//tools/workspace/pycodestyle:pycodestyle.BUILD.bazel",  # noqa
        )

    if "buildifier" in include:
        buildifier_repository(
            name = "buildifier",
        )

# No repositories for `clang-format`. It is used via the system.
