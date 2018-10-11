# -*- python -*-

load(":http_archive_python.bzl", "http_archive_python")
load("@drake//tools/workspace:github.bzl", "github_archive")
load("@drake//tools/workspace:pypi.bzl", "pypi_archive")

def sphinx_repository(name, mirrors = None):
    github_archive(
        name = name,
        repository = "sphinx-doc/sphinx",
        # Needs `typing` and `imagesize`...
        commit = "v1.5.1", #"v1.8.0",
        sha256 = "e6f4fbb8dccd6b9f3f02c82c353bdd30792434a59a04767fd02e3d1dc109d572",
        build_file = "@drake//tools/workspace/sphinx:package.BUILD.bazel",
        mirrors = mirrors,
        patch_cmds = [
            "rm -rf tests",
        ],
        http_archive = http_archive_python,
    )

    pypi_archive(
        name = name + "_typing",
        package = "typing",
        version = "3.6.6",
        build_file_content = """
py_library(
    name = "typing",
    srcs = ["src/typing.py"],
    imports = ["src"],
    visibility = ["//visibility:public"],
)
""",
        sha256 = "4027c5f6127a6267a435201981ba156de91ad0d1d98e9ddc2aa173453453492d",  # noqa
        mirrors = mirrors,
    )

    pypi_archive(
        name = name + "_imagesize",
        package = "imagesize",
        version = "1.0.0",
        build_file_content = """
py_library(
    name = "imagesize",
    srcs = ["imagesize.py"],
    imports = ["."],
    visibility = ["//visibility:public"],
)
""",
        sha256 = "5b326e4678b6925158ccc66a9fa3122b6106d7c876ee32d7de6ce59385b96315",  # noqa
        mirrors = mirrors,
    )
