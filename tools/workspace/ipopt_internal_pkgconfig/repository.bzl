load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def ipopt_internal_pkgconfig_repository(
        name,
        licenses = [
            "reciprocal",  # CPL-1.0
        ],
        modname = "ipopt",
        pkg_config_paths = [],
        homebrew_subdir = "opt/ipopt/lib/pkgconfig",
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        # When using ipopt from pkg-config, there is nothing to install.
        build_epilog = """
load("@drake//tools/install:install.bzl", "install")
install(name = "install")
""",
        **kwargs
    )
