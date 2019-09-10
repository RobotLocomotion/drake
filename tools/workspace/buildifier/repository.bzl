# -*- mode: python -*-
# vi: set ft=python :

"""
Downloads a precompiled version of buildifier and makes it available to the
WORKSPACE.

Example:
    WORKSPACE:
        load("@drake//tools/workspace:mirrors.bzl", "DEFAULT_MIRRORS")
        load("@drake//tools/workspace/buildifier:repository.bzl", "buildifier_repository")  # noqa
        buildifier_repository(name = "foo", mirrors = DEFAULT_MIRRORS)

    BUILD:
        sh_binary(
            name = "foobar",
            srcs = ["bar.sh"],
            data = ["@foo//:buildifier"],
        )

Argument:
    name: A unique name for this rule.
"""

load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)

    if os_result.error != None:
        fail(os_result.error)

    version = "0.29.0"

    if os_result.is_macos:
        filename = "buildifier.mac"
        sha256 = "9b108decaa9a624fbac65285e529994088c5d15fecc1a30866afc03a48619245"  # noqa
    elif os_result.is_ubuntu:
        filename = "buildifier"
        sha256 = "4c985c883eafdde9c0e8cf3c8595b8bfdf32e77571c369bf8ddae83b042028d6"  # noqa
    else:
        fail("Operating system is NOT supported", attr = os_result)

    urls = [
        x.format(version = version, filename = filename)
        for x in repository_ctx.attr.mirrors.get("buildifier")
    ]
    output = repository_ctx.path("buildifier")

    repository_ctx.download(urls, output, sha256, executable = True)

    content = """# -*- python -*-

licenses(["notice"])  # Apache-2.0

exports_files(
    ["buildifier"],
)
"""

    repository_ctx.file("BUILD.bazel", content, executable = False)

buildifier_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
