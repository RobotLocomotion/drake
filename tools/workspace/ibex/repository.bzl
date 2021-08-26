# -*- mode: python -*-

load(
    "@drake//tools/workspace:deb.bzl",
    "setup_new_deb_archive",
)
load(
    "@drake//tools/workspace:execute.bzl",
    "execute_or_fail",
)
load(
    "@drake//tools/workspace:os.bzl",
    "determine_os",
)
load(
    "@drake//tools/workspace:pkg_config.bzl",
    "setup_pkg_config_repository",
)
load(":version.bzl", "IBEX_VERSION")

def _rename_so(repo_ctx, directory, old_name):
    # Rename directory/old_name to be -ldrake_foo instead of -lfoo.
    old_name[:3] == "lib" or fail("Bad library name " + old_name)
    execute_or_fail(repo_ctx, [
        "mv",
        directory + "/lib" + old_name[3:],
        directory + "/libdrake_" + old_name[3:],
    ])

def _impl(repo_ctx):
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)
    if os_result.is_macos:
        result = setup_pkg_config_repository(repo_ctx)
        if result.error != None:
            fail("Unable to complete setup for @{} repository: {}".format(
                # (forced line break)
                repo_ctx.name,
                result.error,
            ))
        return
    if os_result.ubuntu_release not in ["18.04", "20.04"]:
        fail("Operating system is NOT supported {}".format(os_result))
    result = setup_new_deb_archive(repo_ctx)
    if result.error != None:
        fail("Unable to complete setup for @{} repository: {}".format(
            # (forced line break)
            repo_ctx.name,
            result.error,
        ))

    # Avoid using upstream library names for our custom build.
    _rename_so(
        repo_ctx,
        "opt/libibex/{}/lib".format(IBEX_VERSION),
        "libibex.so",
    )

ibex_repository = repository_rule(
    # TODO(jamiesnape): Pass down licenses to setup_pkg_config_repository.
    attrs = {
        "modname": attr.string(default = "ibex"),
        # The next two attributes are only used for macOS.  They are documented
        # in the pkg_config_repository rule.
        "pkg_config_paths": attr.string_list(
            default = [
                "/usr/local/opt/clp/lib/pkgconfig",
                "/usr/local/opt/coinutils/lib/pkgconfig",
                "/usr/local/opt/ibex@{}/share/pkgconfig".format(IBEX_VERSION),
            ],
        ),
        # On macOS we are using IBEX from homebrew, so Drake's installation
        # script doesn't need to do anything extra.
        "build_epilog": attr.string(
            default = "filegroup(name = \"install\")  # Nothing.",
        ),
        # All of the below attributes are only used for Ubuntu.  They are
        # documented in the new_deb_archive rule.
        "mirrors": attr.string_list(
            default = [
                "https://drake-apt.csail.mit.edu/bionic/pool/main",
                "https://s3.amazonaws.com/drake-apt/bionic/pool/main",
            ],
        ),
        "filenames": attr.string_list(
            default = [
                "libi/libibex-dev/libibex-dev_{}.20210826124156.git26eeeaae51b0f1518cbab9751c872b83801dbec8~18.04_amd64.deb".format(IBEX_VERSION),  # noqa
            ],
        ),
        "sha256s": attr.string_list(
            default = [
                "058ba0d538927c0e25b79cd73906abb1a15dc7ffef4f8407d8c6ae4add940e22",  # noqa
            ],
        ),
        "build_file": attr.label(
            default = "@drake//tools/workspace/ibex:package-ubuntu.BUILD.bazel",  # noqa
        ),
    },
    configure = True,
    implementation = _impl,
)

"""Provides a library target for @ibex//:ibex.  On macOS, uses homebrew; on
Ubuntu, downloads a *.deb file and unpacks it into the workspace.
"""
