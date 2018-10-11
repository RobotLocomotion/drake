# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")
load(
    "@bazel_tools//tools/build_defs/repo:http.bzl",
    # HACK
    "patch",
    "workspace_and_buildfile",
    "http_archive",
    # "_http_archive_attrs",
)

# https://github.com/bazelbuild/bazel/blob/09f7cbc/tools/build_defs/repo/http.bzl

def _extract(ctx):
    archive = "_archive.tar.gz"
    script = "_extract.py"
    ctx.symlink(Label("@drake//tools/workspace/sphinx:extract_utf8_robust.py"), script)
    # print(dir(ctx))
    # fail("")
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
        print(result.stdout)
        print(result.stderr)
        fail("Bad")
    result = ctx.execute(["rm", script, archive])
    if result.return_code:
        print(result.stdout)
        print(result.stderr)
        fail("Bad")

def _hack_http_archive_impl(ctx):
    """Implementation of the http_archive rule."""
    if not ctx.attr.url and not ctx.attr.urls:
        fail("At least one of url and urls must be provided")
    if ctx.attr.build_file and ctx.attr.build_file_content:
        fail("Only one of build_file and build_file_content can be provided.")

    all_urls = []
    if ctx.attr.urls:
        all_urls = ctx.attr.urls
    if ctx.attr.url:
        all_urls = [ctx.attr.url] + all_urls
    ctx.download(
        all_urls,
        "_archive.tar.gz",
        ctx.attr.sha256,
        False,
    )
    _extract(ctx)
    patch(ctx)
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
    "patches": attr.label_list(default = []),
    "patch_tool": attr.string(default = "patch"),
    "patch_args": attr.string_list(default = ["-p0"]),
    "patch_cmds": attr.string_list(default = []),
    "workspace_file": attr.label(allow_single_file = True),
    "workspace_file_content": attr.string(),
}

_hack_http_archive = repository_rule(
    implementation = _hack_http_archive_impl,
    attrs = _http_archive_attrs,
)

def sphinx_repository(name, mirrors = None):
    github_archive(
        name = name,
        repository = "sphinx-doc/sphinx",
        # Needs `typing` and `imagesize`...
        commit = "v1.5.0", #"v1.8.0",
        sha256 = "0" * 64,
        build_file = "@drake//tools/workspace/sphinx:package.BUILD.bazel",
        mirrors = mirrors,
        patch_cmds = [
            "rm -rf tests",
        ],
        http_archive = _hack_http_archive,
    )

    # github_archive(
    #     name = name,
    #     repository = "rtfd/sphinx_rtd_theme",
    #     commit = "

    # which_repository(
    #     name = name,
    #     command = "sphinx-build",
    # )
