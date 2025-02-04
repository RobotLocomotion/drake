load("//tools/workspace:alias.bzl", "alias_repository")

def zlib_repository(name):
    alias_repository(
        name = name,
        aliases = {"zlib": "@drake//tools/workspace/zlib"},
    )
