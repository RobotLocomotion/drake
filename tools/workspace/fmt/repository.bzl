# -*- python -*-

load("@drake//tools/workspace:os.bzl", "determine_os")
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
        # On macOS, we use fmt from homebrew via pkg-config.
        error = setup_pkg_config_repository(repo_ctx).error
    elif os_result.is_manylinux:
        # Compile from downloaded github sources.
        error = setup_github_repository(repo_ctx).error
    elif os_result.ubuntu_release == "18.04":
        # On Ubuntu 18.04, the host-provided spdlog is way too old so we can't
        # use its bundled fmt, and there is no other fmt package available, so
        # we'll recompile it from downloaded github sources.
        error = setup_github_repository(repo_ctx).error
    elif os_result.ubuntu_release == "20.04":
        # On Ubuntu 20.04 we're using the host-provided spdlog which uses a
        # bundled fmt, so we'll have to reuse that same bundle for ourselves.
        repo_ctx.symlink("/usr/include/spdlog/fmt/bundled", "include/fmt")
        repo_ctx.symlink("include/fmt/LICENSE.rst", "LICENSE.rst")
        repo_ctx.symlink(
            Label("@drake//tools/workspace/fmt:package.BUILD.bazel"),
            "BUILD.bazel",
        )
        error = None
    else:
        fail("Unsupported OS")
    if error != None:
        fail(error)

fmt_repository = repository_rule(
    attrs = {
        # The next two attributes are used only when we take the branch for
        # setup_pkg_config_repository in the above logic.
        "modname": attr.string(
            default = "fmt",
        ),
        "build_epilog": attr.string(
            # When using fmt from pkg-config, there is nothing to install.
            default = """
load("@drake//tools/install:install.bzl", "install")
install(name = "install")
            """,
        ),
        # The remaining attributes are used only when we take the branch for
        # setup_github_repository in the above logic.
        "repository": attr.string(
            default = "fmtlib/fmt",
        ),
        "commit": attr.string(
            # Per https://github.com/gabime/spdlog/releases/tag/v1.5.0 this is
            # the bundled version we should pin, in cases where we're building
            # from source instead of using the host version.
            default = "6.1.2",
        ),
        "commit_pin": attr.int(
            # Per the comment on "commit", above.
            default = 1,
        ),
        "sha256": attr.string(
            default = "1cafc80701b746085dddf41bd9193e6d35089e1c6ec1940e037fcb9c98f62365",  # noqa
        ),
        "build_file": attr.label(
            default = "@drake//tools/workspace/fmt:package.BUILD.bazel",
        ),
        "extra_strip_prefix": attr.string(),
        "mirrors": attr.string_list_dict(),
    },
    local = True,
    configure = True,
    implementation = _impl,
)
