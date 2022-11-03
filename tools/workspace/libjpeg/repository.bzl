# -*- python -*-

load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)

    if os_result.error != None:
        fail(os_result.error)

    noarch_hdrs = ["jerror.h", "jmorecfg.h", "jpegint.h", "jpeglib.h"]

    if os_result.is_macos:
        libdir = "{}/opt/jpeg-turbo/lib".format(os_result.homebrew_prefix)
        repository_ctx.symlink(
            "{}/opt/jpeg-turbo/include".format(os_result.homebrew_prefix),
            "include",
        )
    elif os_result.is_manylinux or os_result.is_macos_wheel:
        libdir = "/opt/drake-dependencies/lib"
        for hdr in noarch_hdrs + ["jconfig.h"]:
            repository_ctx.symlink(
                "/opt/drake-dependencies/include/{}".format(hdr),
                "include/{}".format(hdr),
            )
    elif os_result.is_ubuntu:
        libdir = "/usr/lib/x86_64-linux-gnu"
        for hdr in noarch_hdrs:
            repository_ctx.symlink(
                "/usr/include/{}".format(hdr),
                "include/{}".format(hdr),
            )
        repository_ctx.symlink(
            "/usr/include/x86_64-linux-gnu/jconfig.h",
            "include/jconfig.h",
        )
    else:
        fail("Operating system is NOT supported {}".format(os_result))

    # Declare the libdir.
    repository_ctx.file(
        "vars.bzl",
        content = "LIBDIR = \"{}\"\n".format(libdir),
        executable = False,
    )

    # Add the BUILD file.
    repository_ctx.symlink(
        Label("@drake//tools/workspace/libjpeg:package.BUILD.bazel"),
        "BUILD.bazel",
    )

libjpeg_repository = repository_rule(
    local = True,
    configure = True,
    implementation = _impl,
)
