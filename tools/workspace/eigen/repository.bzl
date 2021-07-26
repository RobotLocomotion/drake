# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")

def eigen_repository(name):
    new_git_repository(
        name = name,
        remote = "https://gitlab.com/libeigen/eigen.git",
        branch = "3.4",
        patches = [
            # Taken from https://gitlab.com/libeigen/eigen/-/merge_requests/562.  # noqa
            # Once that MR is accepted, we can remove our copy of the patch.
            "@drake//tools/workspace/eigen:68d4a63e6ad812664686dc40a75e1528c308842e.patch",  # noqa
        ],
        build_file_content = """
cc_library(
    name = "eigen",
    hdrs = glob(["Eigen/**", "unsupported/Eigen/**"]),
    includes = ["."],
    visibility = ["//visibility:public"],
)
""",
    )
