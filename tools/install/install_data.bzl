load("//tools/install:install.bzl", "install")

def install_data(
        name,
        data = [],
        visibility = None):
    """A touch of sugar around install() to add a default `data_test` based on
    the current native.package_name from whichever package invokes this macro.
    """
    install(
        name = name,
        data = data,
        data_dest = "share/drake/" + native.package_name(),
        visibility = visibility,
    )
