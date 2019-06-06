# -*- mode: python -*-

load("@drake//tools/workspace:os.bzl", "os_specific_alias_repository")

def lapack_repository(name):
    os_specific_alias_repository(
        name = name,
        mapping = {
            "macOS default": ["lapack=@openblas"],
            "Ubuntu default": ["lapack=@liblapack"],
        },
    )
