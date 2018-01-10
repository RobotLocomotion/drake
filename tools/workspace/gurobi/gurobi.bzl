# -*- python -*-
# This is a Bazel repository_rule for the Gurobi solver.  See
# https://www.bazel.io/versions/master/docs/skylark/repository_rules.html

load("@drake//tools/workspace:os.bzl", "determine_os")

# Ubuntu only: GUROBI_PATH should be the linux64 directory in the Gurobi 7.5.2
# release.
def _gurobi_impl(repository_ctx):
    os_result = determine_os(repository_ctx)
    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        gurobi_path = "/Library/gurobi752/mac64"
        repository_ctx.symlink(gurobi_path, "gurobi-distro")
        warning = "Gurobi 7.5.2 is not installed."

        repository_ctx.file("empty.cc", executable = False)
        srcs = ["empty.cc"]

        lib_path = repository_ctx.path("gurobi-distro/lib")
        linkopts = [
            "-L{}".format(lib_path),
            "-lgurobi75",
        ]
    else:
        gurobi_path = repository_ctx.os.environ.get("GUROBI_PATH", "")
        repository_ctx.symlink(
            gurobi_path or "/MISSING_GUROBI_PATH",
            "gurobi-distro")

        if not gurobi_path:
            warning_detail = "GUROBI_PATH is empty or unset"
        else:
            warning_detail = "GUROBI_PATH=%s is invalid" % gurobi_path
        warning = (
            "gurobi.bzl: The saved value of " + warning_detail + "; " +
            "export GUROBI_PATH to the correct value.")

        # In the Gurobi package, libgurobi75.so is just a symlink to
        # libgurobi.so.7.5.2. However, if you use libgurobi.so.7.5.2 in srcs,
        # executables that link this library will be unable to find it at
        # runtime in the Bazel sandbox, because the NEEDED statements in the
        # executable will not square with the RPATH statements. I don't really
        # know why this happens, but I suspect it might be a Bazel bug.
        srcs = ["gurobi-distro/lib/libgurobi75.so"]

        linkopts = ["-pthread"]

    file_content = """# -*- python -*-

package(default_visibility = ["//visibility:public"])

GUROBI_HDRS = glob([
    "gurobi-distro/include/gurobi_c.h",
    "gurobi-distro/include/gurobi_c++.h",
])

print("{warning}") if not GUROBI_HDRS else cc_library(
    name = "gurobi",
    srcs = {srcs},
    hdrs = GUROBI_HDRS,
    includes = ["gurobi-distro/include"],
    linkopts = {linkopts},
)
""".format(warning = warning, srcs = srcs, linkopts = linkopts)

    if os_result.is_macos:
        file_content += """
load("@drake//tools/install:install.bzl", "install")

install(name = "install")
"""
    else:
        file_content += """
load("@drake//tools/install:install.bzl", "install", "install_files")

install_files(
    name = "install_libraries",
    dest = ".",
    files = [
        "gurobi-distro/lib/libgurobi.so.7.5.2",
        "gurobi-distro/lib/libgurobi75.so",
    ],
    strip_prefix = ["gurobi-distro"],
    visibility = ["//visibility:private"],
)

install(
   name = "install",
   docs = ["gurobi-distro/EULA.pdf"],
   doc_strip_prefix = ["gurobi-distro"],
   deps = [":install_libraries"],
)
"""

    repository_ctx.file("BUILD", content = file_content, executable = False)

gurobi_repository = repository_rule(
    environ = ["GUROBI_PATH"],
    local = True,
    implementation = _gurobi_impl,
)
