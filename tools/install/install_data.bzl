# -*- python -*-

load("@drake//tools/install:install.bzl", "install")

def install_data(
        name,
        data = [],
        visibility = None):
    install(
        name = name,
        data = data,
        data_dest = "share/drake/" + native.package_name(),
        visibility = visibility,
    )
