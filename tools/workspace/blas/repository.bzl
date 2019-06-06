# -*- mode: python -*-

load("@drake//tools/workspace:os.bzl", "os_specific_alias_repository")

def blas_repository(name):
    os_specific_alias_repository(
        name = name,
        mapping = {
            "macOS default": ["blas=@openblas"],
            "Ubuntu default": ["blas=@libblas"],
        },
    )
