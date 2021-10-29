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
        # On macOS, we use spdlog from homebrew via pkg-config.
        error = setup_pkg_config_repository(repo_ctx).error
    elif os_result.is_manylinux:
        # Compile it from downloaded github sources.
        error = setup_github_repository(repo_ctx).error
    elif os_result.ubuntu_release == "18.04":
        # On Ubuntu 18.04, the host-provided spdlog is way too old.  Instead,
        # we'll recompile it from downloaded github sources.
        error = setup_github_repository(repo_ctx).error
    else:
        # On Ubuntu 20.04, we use the host-provided spdlog via pkg-config.
        error = setup_pkg_config_repository(repo_ctx).error
    if error != None:
        fail(error)

spdlog_repository = repository_rule(
    attrs = {
        # The next two attributes are used only when we take the branch for
        # setup_pkg_config_repository in the above logic.
        "modname": attr.string(
            default = "spdlog",
        ),
        "extra_defines": attr.string_list(
            default = ["HAVE_SPDLOG"],
        ),
        "extra_deps": attr.string_list(
            default = ["@fmt"],
        ),
        "build_epilog": attr.string(
            # When using spdlog from pkg-config, there is nothing to install.
            default = """
load("@drake//tools/install:install.bzl", "install")
install(name = "install")
            """,
        ),
        # The remaining attributes are used only when we take the branch for
        # setup_github_repository in the above logic.
        "repository": attr.string(
            default = "gabime/spdlog",
        ),
        "commit": attr.string(
            # Here, we elect to use the same version as Ubuntu 20.04, even
            # though it is not the newest revision.  Sticking with a single,
            # older revision helps reduce spurious CI failures.
            #
            # In the unlikely event that you update the version here, fix up
            # the two spdlog-*.cmake files in this directory, and revisit all
            # of the version compatibility matrix in spdlog/repository.bzl.
            default = "v1.5.0",
        ),
        "commit_pin": attr.int(
            # Per the comment on "commit", above.
            default = 1,
        ),
        "sha256": attr.string(
            default = "b38e0bbef7faac2b82fed550a0c19b0d4e7f6737d5321d4fd8f216b80f8aee8a",  # noqa
        ),
        "build_file": attr.label(
            default = "@drake//tools/workspace/spdlog:package.BUILD.bazel",
        ),
        "extra_strip_prefix": attr.string(),
        "mirrors": attr.string_list_dict(),
    },
    local = True,
    configure = True,
    implementation = _impl,
)
