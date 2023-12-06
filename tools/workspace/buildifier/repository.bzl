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

load(
    "//tools/workspace:github.bzl",
    "github_format_urls",
    "github_release",
    "setup_github_repository",
)

def _download(
        repository_ctx,
        repository,
        commit,
        mirrors,
        strip_prefix):
    # Determine which binary to fetch.
    os_name = repository_ctx.os.name
    if os_name == "mac os x":
        os_name = "darwin"
    os_arch = repository_ctx.os.arch
    if os_arch == "aarch64":
        os_arch = "arm64"
    filename = "buildifier-{}-{}".format(os_name, os_arch)

    sha256 = repository_ctx.attr.sha256s.get(filename, "0" * 64)
    urls = github_format_urls(
        repository = repository,
        commit = commit,
        mirrors = mirrors,
        substitutions = {
            "release": commit,
            "filename": filename,
        },
    )

    # Fetch the binary from mirrors.
    repository_ctx.download(
        url = urls,
        output = repository_ctx.path("buildifier"),
        sha256 = sha256 or "0" * 64,
        executable = True,
    )

    return sha256, urls

def _impl(repository_ctx):
    result = setup_github_repository(
        repository_ctx,
        downloader = _download,
        repository_type = "github_release",
    )
    if result.error != None:
        fail("Unable to complete setup for " +
             "@{} repository: {}".format(
                 repository_ctx.name,
                 result.error,
             ))

def buildifier_repository(
        name,
        mirrors = None):
    github_release(
        name = name,
        implementation = _impl,
        repository = "bazelbuild/buildtools",
        commit = "v6.4.0",
        sha256s = {
            "buildifier-darwin-amd64": "eeb47b2de27f60efe549348b183fac24eae80f1479e8b06cac0799c486df5bed",  # noqa
            "buildifier-darwin-arm64": "fa07ba0d20165917ca4cc7609f9b19a8a4392898148b7babdf6bb2a7dd963f05",  # noqa
            "buildifier-linux-amd64": "be63db12899f48600bad94051123b1fd7b5251e7661b9168582ce52396132e92",  # noqa
            "buildifier-linux-arm64": "18540fc10f86190f87485eb86963e603e41fa022f88a2d1b0cf52ff252b5e1dd",  # noqa
        },
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
