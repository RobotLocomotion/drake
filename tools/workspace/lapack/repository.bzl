load("//tools/workspace:alias.bzl", "alias_repository")

def lapack_repository(name):
    alias_repository(
        name = name,
        aliases = {"lapack": "@drake//tools/workspace/lapack"},
    )
