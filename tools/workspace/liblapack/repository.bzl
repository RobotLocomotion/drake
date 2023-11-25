load("//tools/workspace:pkg_config.bzl", "pkg_config_repository")

def liblapack_repository(name):
    # Note that @liblapack is typically only available on Linux.
    # See drake/tools/workspace/lapack for the OS-specific selection policy.
    pkg_config_repository(
        name = name,
        licenses = ["notice"],  # BSD-3-Clause
        modname = "lapack",
        extra_deps = [
            "@gfortran//:runtime",
            "@libblas",
        ],
    )
