load("//tools/workspace:alias.bzl", "alias_repository")

def zlib_repository(name, _legacy_workspace = True):
    actual = "@drake//tools/workspace/zlib"
    if _legacy_workspace:
        actual += ":hardcoded"
    alias_repository(
        name = name,
        aliases = {"zlib": actual},
    )
