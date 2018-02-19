# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def yaml_cpp_repository(
        name,
        modname = "yaml-cpp",
        atleast_version = "0.5.2",
        extra_deps = ["@boost//:boost_headers"],
        pkg_config_paths = ["/usr/local/opt/yaml-cpp/lib/pkgconfig"],
        **kwargs):
    pkg_config_repository(
        name = name,
        modname = modname,
        atleast_version = atleast_version,
        extra_deps = extra_deps,
        pkg_config_paths = pkg_config_paths,
        **kwargs)
