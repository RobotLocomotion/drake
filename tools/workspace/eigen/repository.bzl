# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def eigen_repository(
        name,
        **kwargs):
    # This is a git commit sha near the tip of master.
    commit = "12ad99ce60ae7526ca831e845601130914042070"
    http_archive(
        name = name,
        url = "{}/-/archive/{}/eigen-{}.tar.gz".format(
            "https://gitlab.com/libeigen/eigen",
            commit,
            commit,
        ),
        sha256 = "7c224d88a90e02595eb75ab1105e788ff6655718bab694fd2c8707734a407933",  # noqa
        strip_prefix = "eigen-{}".format(commit),
        build_file_content = """
cc_library(
    name = "eigen",
    hdrs = glob(
        include = [
            "Eigen/*",
            "Eigen/**/*.h",
            "unsupported/Eigen/*",
            "unsupported/Eigen/CXX11/Tensor",
            "unsupported/Eigen/**/*.h",
        ],
    ),
    includes = ["."],
    visibility = ["//visibility:public"],
)
""",
    )
