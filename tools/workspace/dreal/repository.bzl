# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def dreal_repository(
        name,
        modname = "dreal",
        # Since dReal and its dependency IBEX are installed under
        # `/opt/<package_name>/<version_number>/`, it is necessary to keep the
        # following versions in sync with the ones in
        # `setup/ubuntu/16.04/install_prereqs.sh` script.
        pkg_config_paths = [
            "/opt/dreal/4.18.01.3/lib/pkgconfig",
            "/opt/libibex/2.6.5/share/pkgconfig",
            "/usr/local/opt/clp/lib/pkgconfig",
            "/usr/local/opt/coinutils/lib/pkgconfig",
            "/usr/local/opt/dreal/lib/pkgconfig",
            "/usr/local/opt/ibex@2.6.5/share/pkgconfig",
            "/usr/local/opt/nlopt/lib/pkgconfig",
        ],
        **kwargs):
    pkg_config_repository(
        name = name,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        **kwargs)
