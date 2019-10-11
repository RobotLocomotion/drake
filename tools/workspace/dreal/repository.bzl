# -*- mode: python -*-

load(
    "@drake//tools/workspace:os.bzl",
    "determine_os",
)
load(
    "@drake//tools/workspace:github.bzl",
    "setup_github_repository",
)
load(
    "@drake//tools/workspace:pkg_config.bzl",
    "setup_pkg_config_repository",
)

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
    result = setup_github_repository(repo_ctx)
    if result.error != None:
        fail("Unable to complete setup for @{} repository: {}".format(
            # (forced line break)
            repo_ctx.name,
            result.error,
        ))

dreal_repository = repository_rule(
    # TODO(jamiesnape): Pass down licenses to setup_pkg_config_repository.
    attrs = {
        "modname": attr.string(default = "dreal"),
        # The next two attributes are only used for macOS.  They are documented
        # in the pkg_config_repository rule.
        "pkg_config_paths": attr.string_list(
            default = [
                "/usr/local/opt/dreal/lib/pkgconfig",
            ],
        ),
        # On macOS we are using dReal from homebrew, so Drake's installation
        # script doesn't need to do anything extra.
        "build_epilog": attr.string(
            default = "filegroup(name = \"install\")  # Nothing.",
        ),
        # All of the below attributes are only used for Ubuntu.  They are
        # documented in the setup_github_archive rule.
        "repository": attr.string(
            default = "dreal/dreal4",
        ),
        "commit": attr.string(
            default = "4.19.10.2",
        ),
        "sha256": attr.string(
            default = "0488b4da501bace1fc2e2418ce7f8b1d28b6bd798923c2619756a5e905cd83a5",  # noqa
        ),
        "build_file": attr.label(
            default = None,
        ),
        "extra_strip_prefix": attr.string(
            default = "",
        ),
        "mirrors": attr.string_list_dict(
            mandatory = True,
            allow_empty = False,
        ),
    },
    implementation = _impl,
)

"""Provides a library target for @dreal//:dreal.  On macOS, uses homebrew; on
Ubuntu, compiles from source.
"""
