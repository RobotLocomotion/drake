# -*- python -*-

load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)

    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        repository_ctx.symlink("/usr/local/opt/jpeg/include", "include")
        repository_ctx.symlink(
            Label("@drake//tools/workspace/libjpeg:package-macos.BUILD.bazel"),
            "BUILD.bazel",
        )
    elif os_result.is_ubuntu:
        for hdr in ["jerror.h", "jmorecfg.h", "jpegint.h", "jpeglib.h"]:
            repository_ctx.symlink(
                "/usr/include/{}".format(hdr),
                "include/{}".format(hdr),
            )
        repository_ctx.symlink(
            "/usr/include/x86_64-linux-gnu/jconfig.h",
            "include/jconfig.h",
        )
        repository_ctx.symlink(
            Label(
                "@drake//tools/workspace/libjpeg:package-ubuntu.BUILD.bazel",
            ),
            "BUILD.bazel",
        )
    else:
        fail("Operating system is NOT supported", attr = os_result)

libjpeg_repository = repository_rule(
    local = True,
    configure = True,
    implementation = _impl,
)
