# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def freetype2_repository(
        name,
        # BSD-2-Clause AND BSD-3-Clause AND (FTL OR GPL-2.0+)
        licenses = ["notice"],
        modname = "freetype2",
        pkg_config_paths = ["/usr/local/opt/freetype/lib/pkgconfig"],
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        build_epilog = "print('DRAKE DEPRECATED: The @freetype2 external is being removed from Drake on or after 2020-12-01.  Downstream projects should add it to their own WORKSPACE if needed.')",  # noqa
        **kwargs
    )
