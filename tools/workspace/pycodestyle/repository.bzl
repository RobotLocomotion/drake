load("@drake//tools/workspace:github.bzl", "github_archive")

def pycodestyle_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "PyCQA/pycodestyle",
        commit = "2.11.0",
        sha256 = "757a3dba55dce2ae8b01fe7b46c20cd1e4c0fe794fe6119bce66b942f35e2db0",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
