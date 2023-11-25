load("//tools/workspace:pkg_config.bzl", "pkg_config_repository")

def libblas_repository(name):
    # Note that @libblas is typically only available on Linux.
    # See drake/tools/workspace/blas for the OS-specific selection policy.
    pkg_config_repository(
        name = name,
        licenses = ["notice"],  # BSD-3-Clause
        modname = "blas",
    )
