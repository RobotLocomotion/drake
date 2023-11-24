load("//tools/workspace:pkg_config.bzl", "pkg_config_repository")

def ipopt_internal_pkgconfig_repository(
        name,
        licenses = [
            "reciprocal",  # CPL-1.0
        ],
        modname = "ipopt",
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        # When using ipopt from pkg-config, there is nothing to install.
        build_epilog = """
load("@drake//tools/install:install.bzl", "install")
install(name = "install")
""",
        **kwargs
    )
