load("//tools/workspace:pkg_config.bzl", "pkg_config_repository")

def libblas_repository(name):
    # Note that @libblas is typically only available on Linux.
    # See drake/tools/workspace/blas for the OS-specific selection policy.
    # Note that on Debian and Ubuntu in particular, @libblas refers to the
    # implementation chosen by the Ubuntu alternatives system, which might be
    # one of any number of different vendor implementations.
    pkg_config_repository(
        name = name,
        licenses = ["notice"],  # BSD-3-Clause
        modname = "blas",
    )
