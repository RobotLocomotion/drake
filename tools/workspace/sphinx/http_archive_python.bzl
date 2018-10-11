load(
    "@bazel_tools//tools/build_defs/repo:http.bzl",
    "workspace_and_buildfile",
)

def _extract(ctx, archive):
    script = "_extract.py"
    ctx.symlink(
        Label("@drake//tools/workspace/sphinx:extract_utf8_robust.py"), script)
    args = [
        "/usr/bin/python2", script,
        "--strip_prefix", ctx.attr.strip_prefix,
        "--output_dir", ".",
    ]
    for cmd in ctx.attr.patch_cmds:
        args += ["--patch_cmd", cmd]
    args += [archive]
    result = ctx.execute(args)
    if result.return_code:
        fail("Exit code {}:\n{}\n{}".format(
            result.return_code, result.stdout, result.stderr))
    result = ctx.execute(["rm", script, archive])
    if result.return_code:
        fail("Exit code {}:\n{}\n{}".format(
            result.return_code, result.stdout, result.stderr))

def _impl(ctx):
    # Using API from: https://github.com/bazelbuild/bazel/blob/09f7cbc/tools/build_defs/repo/http.bzl  # noqa
    if not ctx.attr.url and not ctx.attr.urls:
        fail("At least one of url and urls must be provided")
    if ctx.attr.build_file and ctx.attr.build_file_content:
        fail("Only one of build_file and build_file_content can be provided.")

    all_urls = []
    if ctx.attr.urls:
        all_urls = ctx.attr.urls
    if ctx.attr.url:
        all_urls = [ctx.attr.url] + all_urls
    archive = "_archive.tar.gz"
    ctx.download(
        all_urls,
        archive,
        ctx.attr.sha256,
        False,
    )
    _extract(ctx, archive)
    workspace_and_buildfile(ctx)

# Copy pasta.
_http_archive_attrs = {
    "url": attr.string(),
    "urls": attr.string_list(),
    "sha256": attr.string(),
    "strip_prefix": attr.string(),
    "type": attr.string(),
    "build_file": attr.label(allow_single_file = True),
    "build_file_content": attr.string(),
    "patch_cmds": attr.string_list(default = []),
    "workspace_file": attr.label(allow_single_file = True),
    "workspace_file_content": attr.string(),
}

"""
Workaround to the following issue using a simple Python extract script:
https://github.com/bazelbuild/bazel/issues/1653
"""

http_archive_python = repository_rule(
    implementation = _impl,
    attrs = _http_archive_attrs,
)
