load("//tools/workspace:pkg_config.bzl", "pkg_config_repository")

def eigen_repository(name):
    pkg_config_repository(
        name = name,
        licenses = [
            "notice",  # BSD-3-Clause
            "reciprocal",  # MPL-2.0
            "unencumbered",  # Public-Domain
        ],
        modname = "eigen3",
        extra_defines = ["EIGEN_MPL2_ONLY"],
    )
