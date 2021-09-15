# -*- mode: python -*-

load("@drake//tools/workspace:os.bzl", "os_specific_alias_repository")

# TLDR: Use this repository (@lapack) instead of @liblapack or @openblas in the
# deps of your target.
#
# Since we use different pkg-config modules offering LAPACK implementations
# from (potentially) different vendors depending on the operating system, this
# alias exists to select the correct operating-system-specific LAPACK
# implementation. On macOS, the @lapack and @openblas targets will always refer
# to OpenBLAS. On Ubuntu, the @lapack and @liblapack targets will depend on a
# LAPACK implementation typically chosen by the Ubuntu alternatives system,
# such as the reference implementation from Netlib, Automatically Tuned Linear
# Algebra Software (ATLAS), and OpenBLAS.
def lapack_repository(name):
    os_specific_alias_repository(
        name = name,
        mapping = {
            "macOS default": ["lapack=@openblas"],
            "Ubuntu default": ["lapack=@liblapack"],
            "manylinux": ["lapack=@liblapack"],
        },
    )
