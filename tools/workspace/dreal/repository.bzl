# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def dreal_repository(name, modname = "dreal", **kwargs):
    pkg_config_repository(name = name,
                          modname = modname,
                          # Since dReal and its dependency IBEX are installed
                          # under `/opt/<package_name>/<version_number>/`, it
                          # is necessary to keep the following versions in
                          # sync with the ones in
                          # `setup/ubuntu/16.04/install_prereqs.sh` script.
                          pkg_config_paths = [
                              # For Ubuntu:
                              "/opt/dreal/4.18.01.3/lib/pkgconfig",
                              "/opt/libibex/2.6.5/share/pkgconfig",
                              # For macOS, we enumerate its complete dependency
                              # information here so that homebrew can find all
                              # it needs:
                              "/usr/local/opt/dreal/lib/pkgconfig",
                              "/usr/local/opt/ibex@2.6.5/share/pkgconfig",
                              "/usr/local/opt/clp/lib/pkgconfig",
                              "/usr/local/opt/coinutils/lib/pkgconfig",
                              "/usr/local/opt/nlopt/lib/pkgconfig",
                          ],
                          **kwargs)
