# -*- python -*-
# This is a Bazel repository_rule for libgfortran.  See
# https://www.bazel.io/versions/master/docs/skylark/repository_rules.html

def _gfortran_impl(repository_ctx):
    """Locate libgfortran.a and wrap it in a cc_library rule."""
    result = repository_ctx.execute(["gfortran",
                                     "--print-file-name=libgfortran.a"])
    warning_detail = ""
    success = True
    if result.return_code != 0:
        success = False
        warning_detail = "Could not --print-file-name. Is gfortran installed?"
    else:
        path = result.stdout.strip()
        if path:
            repository_ctx.symlink(path, "libgfortran.a")
        else:
            success = False
            warning_detail = "Found gfortran but not libgfortran.a. Yikes!"

    if success:
        BUILD = """
        cc_library(
            name = "lib",
            srcs = ["libgfortran.a"],
            hdrs = [],
            visibility = ["//visibility:public"],
        )
        """
    else:
        fail("gfortran.bzl: " + warning_detail)

    BUILD = BUILD.replace("\n        ", "\n")  # Strip leading indentation.
    repository_ctx.file("BUILD", content=BUILD)

gfortran_repository = repository_rule(
    local = True,
    implementation = _gfortran_impl,
)
