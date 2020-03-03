# -*- python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def eigen_repository(
        name,
        licenses = [
            "notice",  # BSD-3-Clause
            "reciprocal",  # MPL-2.0
            "unencumbered",  # Public-Domain
        ],
        modname = "eigen3",
        atleast_version = "3.3.4",
        extra_defines = ["EIGEN_MPL2_ONLY"],
        pkg_config_paths = ["/usr/local/opt/eigen/share/pkgconfig"],
        mirrors = None,
        **kwargs):
    if mirrors != None:
        print("DRAKE DEPRECATED: The @eigen external no longer accepts a mirrors= parameter, because Drake no longer downloads Eigen; this warning will turn into an error or after 2020-06-01.")  # noqa
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        extra_defines = extra_defines,
        pkg_config_paths = pkg_config_paths,
        **kwargs
    )
