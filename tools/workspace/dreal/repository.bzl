# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def dreal_repository(name, modname = "dreal", **kwargs):
    pkg_config_repository(name = name,
                          modname = modname,
                          pkg_config_paths = [
                              "/opt/dreal/4.18.01.3/lib/pkgconfig",
                              "/opt/libibex/2.6.5/share/pkgconfig",
                          ],
                          **kwargs)
