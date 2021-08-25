# -*- mode: python -*-

load("@drake//tools/workspace:os.bzl", "os_specific_alias_repository")

# TLDR: Use this repository (@blas) instead of @libblas or @openblas in the
# deps of your target.
#
# Since we use different pkg-config modules offering BLAS implementations from
# (potentially) different vendors depending on the operating system, this alias
# exists to select the correct operating-system-specific BLAS implementation.
# On macOS, the @blas and @openblas targets will always refer to OpenBLAS. On
# Ubuntu, the @blas and @libblas targets will depend on a BLAS implementation
# typically chosen by the Ubuntu alternatives system, such as the reference
# implementation from Netlib, Automatically Tuned Linear Algebra Software
# (ATLAS), and OpenBLAS.
def blas_repository(name):
    os_specific_alias_repository(
        name = name,
        mapping = {
            "macOS default": ["blas=@openblas"],
            "Ubuntu default": ["blas=@libblas"],
            "manylinux": ["blas=@libblas"],
        },
    )
