# -*- mode: python -*-

load(
    "@drake//tools/workspace:os.bzl",
    "determine_os",
)
load(
    "@drake//tools/workspace:pkg_config.bzl",
    "setup_pkg_config_repository",
)

def _impl(repo_ctx):
    # Only available on Ubuntu. On macOS, no targets should depend on @opengl.
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)
    if os_result.is_ubuntu:
        include = "/usr/include/GL"
        lib = "/usr/lib"

        # Grab the relevant headers.
        hdrs = [
            "freeglut.h",
            "freeglut_ext.h",
            "freeglut_std.h",
            "glut.h",
        ]
        for hdr in hdrs:
            repo_ctx.symlink(include + "/" + hdr, "include/" + hdr)

        # Declare the libdir.
        repo_ctx.file(
            "vars.bzl",
            content = "LIBDIR = \"{}\"\n".format(lib),
            executable = False,
        )

        # Add the BUILD file.
        repo_ctx.symlink(
            Label("@drake//tools/workspace/glut:package.BUILD.bazel"),
            "BUILD.bazel",
        )

    else:
        repo_ctx.symlink(
            Label("@drake//tools/workspace/glut:package-macos.BUILD.bazel"),
            "BUILD.bazel",
        )

glut_repository = repository_rule(
    local = True,
    implementation = _impl,
)
