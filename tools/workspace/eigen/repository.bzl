# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")

def eigen_repository(name):
    new_git_repository(
        name = name,
        remote = "https://gitlab.com/libeigen/eigen.git",
        branch = "3.4",
        build_file_content = """
cc_library(
    name = "eigen",
    hdrs = glob(["Eigen/**", "unsupported/Eigen/**"]),
    includes = ["."],
    visibility = ["//visibility:public"],
)
""",
    )
