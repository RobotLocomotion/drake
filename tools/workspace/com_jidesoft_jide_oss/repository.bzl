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
    # @com_jidesoft_jide_oss.
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_ubuntu:
        repo_ctx.symlink(
            Label("@drake//tools/workspace/com_jidesoft_jide_oss:package-ubuntu.BUILD.bazel"),  # noqa
            "BUILD.bazel",
        )
        repo_ctx.symlink(
            "/usr/share/java/jide-oss.jar",
            "jar/jide-os.jar",
        )
        repo_ctx.symlink(
            "BUILD.bazel",
            "jar/BUILD.bazel",
        )
    else:
        java_import_external(
            name = repo_ctx.name,
            licenses = ["restricted"],  # GPL-2.0 WITH Classpath-exception-2.0
            jar_urls = [
                x.format(fulljar = "com/jidesoft/jide-oss/2.9.7/jide-oss-2.9.7.jar")  # noqa
                for x in repo_ctx.attr.mirrors.get("maven")
            ],
            jar_sha256 = "a2edc2749cf482f6b2b1331f35f0383a1a11c19b1cf6d9a8cf7c69ce4cc8e04b",  # noqa
        )

com_jidesoft_jide_oss_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
