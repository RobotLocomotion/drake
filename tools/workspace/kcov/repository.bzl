# -*- mode: python -*-

load(
    "@drake//tools/workspace:deb.bzl",
    "setup_new_deb_archive",
)
load(
    "@drake//tools/workspace:os.bzl",
    "determine_os",
)

def _impl(repo_ctx):
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)
    if os_result.is_ubuntu:
        result = setup_new_deb_archive(repo_ctx)
        if result.error != None:
            fail("Unable to complete setup for @{} repository: {}".
                 format(repo_ctx.name, result.error))
        repo_ctx.symlink("opt/kcov/34/bin/kcov", "kcov")
    else:
        found = repo_ctx.which("kcov")
        if not found:
            fail("Could not find kcov on PATH={}".format(
                repo_ctx.os.environ["PATH"]))
        repo_ctx.symlink(found, "kcov")
        repo_ctx.symlink(repo_ctx.attr.build_file, "BUILD.bazel")

kcov_repository = repository_rule(
    attrs = {
        # All of the below attributes are only used for Ubuntu.  They are
        # documented in the new_deb_archive rule.
        "mirrors": attr.string_list(
            default = [
                "https://drake-apt.csail.mit.edu/pool/main",
            ],
        ),
        "filenames": attr.string_list(
            default = [
                "k/kcov-34/kcov-34_34-1_amd64.deb",
            ],
        ),
        "sha256s": attr.string_list(
            default = [
                "28eebdf9d103bd9db640ea374ab91ca3196bc743eaba934960f103afbbe868c3",  # noqa
            ],
        ),
        "build_file": attr.label(
            default = "@drake//tools/workspace/kcov:package.BUILD.bazel",  # noqa
        ),
    },
    environ = ["PATH"],
    local = True,
    implementation = _impl,
)

"""Provides a library target for @kcov//:kcov.  On macOS, uses homebrew;
on Ubuntu, downloads a *.deb file and unpacks it into the workspace.
"""
