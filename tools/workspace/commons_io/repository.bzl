# -*- mode: python -*-

load(
    "@drake//tools/workspace:os.bzl",
    "determine_os",
)
load(
    "@bazel_tools//tools/build_defs/repo:java.bzl",
    "java_import_external",
)

def _impl(repo_ctx):
    # Only available on macOS. On Ubuntu, no targets should depend on
    # @commons_io.
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_ubuntu:
        repo_ctx.symlink(
            Label("@drake//tools/workspace/commons_io:package-ubuntu.BUILD.bazel"),  # noqa
            "BUILD.bazel",
        )
        repo_ctx.symlink(
            "/usr/share/java/commons-io.jar",
            "jar/commons-io.jar",
        )
        repo_ctx.symlink(
            "BUILD.bazel",
            "jar/BUILD.bazel",
        )
    else:
        java_import_external(
            name = repo_ctx.name,
            licenses = ["notice"],  # Apache-2.0
            jar_urls = [
                x.format(fulljar = "commons-io/commons-io/1.3.1/commons-io-1.3.1.jar")  # noqa
                for x in repo_ctx.attr.mirrors.get("maven")
            ],
            jar_sha256 = "3307319ddc221f1b23e8a1445aef10d2d2308e0ec46977b3f17cbb15c0ef335b",  # noqa
        )

commons_io_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
