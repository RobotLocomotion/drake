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
load(
    ":versions.bzl",
    "DREAL_VERSION",
    "IBEX_VERSION",
)

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
    _rename_so(
        repo_ctx,
        "opt/dreal/{}/lib".format(DREAL_VERSION),
        "libdreal.so",
    )
    execute_or_fail(repo_ctx, [
        "chmod",
        "a+w",
        "opt/dreal/{}/lib/libdrake_dreal.so".format(DREAL_VERSION),
    ])

    # Our BUILD file declares this dependency with the revised spelling.
    execute_or_fail(repo_ctx, [
        "patchelf",
        "--remove-needed",
        "libibex.so",
        "opt/dreal/{}/lib/libdrake_dreal.so".format(DREAL_VERSION),
    ])

dreal_repository = repository_rule(
    # TODO(jamiesnape): Pass down licenses to setup_pkg_config_repository.
    attrs = {
        "modname": attr.string(default = "dreal"),
        # The next two attributes are only used for macOS.  They are documented
        # in the pkg_config_repository rule.
        "pkg_config_paths": attr.string_list(
            default = [
                # TODO(soonho-tri): Remove the following two lines.
                "/usr/local/opt/clp/lib/pkgconfig",
                "/usr/local/opt/coinutils/lib/pkgconfig",
                "/usr/local/opt/clp@1.17/lib/pkgconfig",
                "/usr/local/opt/dreal/lib/pkgconfig",
                "/usr/local/opt/ibex@{}/share/pkgconfig".format(IBEX_VERSION),
                "/usr/local/opt/nlopt/lib/pkgconfig",
            ],
        ),
        # On macOS we are using dReal from homebrew, so Drake's installation
        # script doesn't need to do anything extra.
        "build_epilog": attr.string(
            default = "filegroup(name = \"install\")  # Nothing.",
        ),
        # All of the below attributes are only used for Ubuntu.  They are
        # documented in the new_deb_archive rule.
        "mirrors": attr.string_list(
            default = [
                "https://drake-apt.csail.mit.edu/xenial/pool/main",
                "https://s3.amazonaws.com/drake-apt/xenial/pool/main",
            ],
        ),
        "filenames": attr.string_list(
            default = [
                "d/dreal/dreal_{}_amd64.deb".format(DREAL_VERSION),
                "libi/libibex-dev/libibex-dev_{}.20190612163212.gitfd0888707728e183c860793f225fd4e5e2ce9d91~16.04_amd64.deb".format(IBEX_VERSION),  # noqa
            ],
        ),
        "sha256s": attr.string_list(
            default = [
                "000b99ad5a86c46eda98d12622688555350d94b24a16941bb0e45c8d2c613952",  # noqa
                "b996e587c9731b90be3d0f31f8e0b5db364a6b7847a8e2207361d753c985cfed",  # noqa
            ],
        ),
        "build_file": attr.label(
            default = "@drake//tools/workspace/dreal:package-ubuntu.BUILD.bazel",  # noqa
        ),
    },
    implementation = _impl,
)

"""Provides a library target for @dreal//:dreal.  On macOS, uses homebrew; on
Ubuntu, downloads *.deb files and unpacks them into the workspace.
"""
