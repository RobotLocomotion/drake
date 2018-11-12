# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _impl(repository_ctx):
    urls = [
        url.format(
            repository = "nlohmann/json",
            release = "v3.4.0",
            filename = "json.hpp",
        )
        for url in repository_ctx.attr.mirrors["github-releases"]
    ]
    repository_ctx.download(
        url = urls,  # noqa
        output = "single_include/nlohmann/json.hpp",
        sha256 = "63da6d1f22b2a7bb9e4ff7d6b255cf691a161ff49532dcc45d398a53e295835f",  # noqa
    )
    repository_ctx.symlink(
        Label("@drake//tools/workspace/json:package.BUILD.bazel"),
        "BUILD.bazel",
    )

json_repository = repository_rule(
    implementation = _impl,
    attrs = dict(
        mirrors = attr.string_list_dict(),
    ),
)
