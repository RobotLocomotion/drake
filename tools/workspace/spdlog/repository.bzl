load("//tools/workspace:github.bzl", "setup_github_repository")
load("//tools/workspace:os.bzl", "is_wheel_build")
load("//tools/workspace:pkg_config.bzl", "setup_pkg_config_repository")

def _impl(repo_ctx):
    if is_wheel_build(repo_ctx):
        # Compile it from downloaded github sources.
        error = setup_github_repository(repo_ctx).error
    else:
        # When not using a wheel build, find spdlog via pkg-config.
        error = setup_pkg_config_repository(repo_ctx).error
    if error != None:
        fail(error)

spdlog_repository = repository_rule(
    attrs = {
        # The next several attributes are used only when we take the branch for
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
            # Here, we elect to use the same version as Ubuntu 22.04, even
            # though it is not the newest revision.  Sticking with a single,
            # older revision helps reduce spurious CI failures.
            default = "v1.9.2",
        ),
        "commit_pin": attr.int(
            # Per the comment on "commit", above.
            default = 1,
        ),
        "sha256": attr.string(
            default = "6fff9215f5cb81760be4cc16d033526d1080427d236e86d70bb02994f85e3d38",  # noqa
        ),
        "build_file": attr.label(
            default = "@drake//tools/workspace/spdlog:package.BUILD.bazel",
        ),
        "extra_strip_prefix": attr.string(),
        "mirrors": attr.string_list_dict(),
    },
    environ = ["DRAKE_OS"],
    local = True,
    configure = True,
    implementation = _impl,
)
