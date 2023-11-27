load("//tools/workspace:pkg_config.bzl", "pkg_config_repository")

def openblas_repository(name):
    # Note that @openblas is typically only available on macOS.
    # See drake/tools/workspace/blas for the OS-specific selection policy.
    pkg_config_repository(
        name = name,
        licenses = ["notice"],  # BSD-3-Clause
        modname = "openblas",
        pkg_config_paths = [
            # TODO(#20581) Homebrew is missing the *.pc file symlink. We'll
            # need to work-around that with some hard-coded paths here.
            "/opt/homebrew/opt/openblas/lib/pkgconfig",
            "/usr/local/opt/openblas/lib/pkgconfig",
        ],
    )
