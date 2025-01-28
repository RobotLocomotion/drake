load("//tools/workspace:alias.bzl", "alias_repository")

def blas_repository(name):
    alias_repository(
        name = name,
        aliases = {"blas": "@drake//tools/workspace/blas"},
    )
