load("//tools/workspace:pkg_config.bzl", "pkg_config_repository")

def liblapack_repository(name):
    # Note that @liblapack is typically only available on Linux.
    # See drake/tools/workspace/lapack for the OS-specific selection policy.
    # Note that on Debian and Ubuntu in particular, @liblapack refers to the
    # implementation chosen by the Ubuntu alternatives system, which might be
    # one of any number of different vendor implementations.
    pkg_config_repository(
        name = name,
        licenses = ["notice"],  # BSD-3-Clause
        modname = "lapack",
        extra_deps = [
            "@gfortran//:runtime",
            "@libblas",
        ],
    )
