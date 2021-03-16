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
    # @net_sf_jchart2d.
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_ubuntu:
        repo_ctx.symlink(
            Label("@drake//tools/workspace/net_sf_jchart2d:package-ubuntu.BUILD.bazel"),  # noqa
            "BUILD.bazel",
        )
        repo_ctx.symlink(
            "/usr/share/java/jchart2d.jar",
            "jar/jchart2d.jar",
        )
        repo_ctx.symlink(
            "BUILD.bazel",
            "jar/BUILD.bazel",
        )
    else:
        repo_ctx.symlink(
            Label("@drake//tools/workspace/net_sf_jchart2d:package-macos.BUILD.bazel"),  # noqa
            "BUILD.bazel",
        )

        # In the unlikely event that you update the version here, verify that
        # the licenses in tools/third_party/jchart2d/LICENSE are still
        # applicable.
        java_import_external(
            name = repo_ctx.name,
            licenses = ["restricted"],  # LGPL-3.0+
            jar_urls = [
                x.format(fulljar = "net/sf/jchart2d/jchart2d/3.3.2/jchart2d-3.3.2.jar")  # noqa
                for x in repo_ctx.attr.mirrors.get("maven")
            ],
            jar_sha256 = "41af674b1bb00d8b89a0649ddaa569df5750911b4e33f89b211ae82e411d16cc",  # noqa
        )

net_sf_jchart2d_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
