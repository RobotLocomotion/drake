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
    # Only available on macOS. On Ubuntu, no targets should depend On
    # @org_apache_xmlgraphics_commons.
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_ubuntu:
        repo_ctx.symlink(
            Label("@drake//tools/workspace/org_apache_xmlgraphics_commons:package-ubuntu.BUILD.bazel"),  # noqa
            "BUILD.bazel",
        )
        repo_ctx.symlink(
            "/usr/share/java/xmlgraphics-commons.jar",
            "jar/xmlgraphics-commons.jar",
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
                x.format(fulljar = "org/apache/xmlgraphics/xmlgraphics-commons/1.3.1/xmlgraphics-commons-1.3.1.jar")  # noqa
                for x in repo_ctx.attr.mirrors.get("maven")
            ],
            jar_sha256 = "7ce0c924c84e2710c162ae1c98f5047d64f528268792aba642d4bae5e1de7181",  # noqa
        )

org_apache_xmlgraphics_commons_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
