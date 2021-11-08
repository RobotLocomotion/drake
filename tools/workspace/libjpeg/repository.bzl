# -*- python -*-

load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)

    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        build_flavor = "macos"
        repository_ctx.symlink("/usr/local/opt/jpeg/include", "include")
    elif os_result.is_ubuntu or os_result.is_manylinux:
        build_flavor = "ubuntu"
        for hdr in ["jerror.h", "jmorecfg.h", "jpegint.h", "jpeglib.h"]:
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

    repository_ctx.symlink(
        Label(
            "@drake//tools/workspace/libjpeg:package-{}.BUILD.bazel".format(
                build_flavor,
            ),
        ),
        "BUILD.bazel",
    )

libjpeg_repository = repository_rule(
    local = True,
    configure = True,
    implementation = _impl,
)
