# -*- mode: python -*-

load(
    "@drake//tools/workspace:os.bzl",
    "determine_os",
)

def _impl(repo_ctx):
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)

    repo_ctx.symlink(
        Label("@drake//tools/workspace/castxml:package.BUILD.bazel"),
        "BUILD.bazel",
    )

    if os_result.is_macos:
        fail("Nah bruh")
    elif os_result.is_ubuntu:
        # v0.3.4: https://data.kitware.com/#item/5ee7eb659014a6d84ec1f25c
        url = "https://data.kitware.com/api/v1/file/5ee7eb659014a6d84ec1f25e/download"
        sha256 = "c3fb619468d20c66d5b56b0f472ceb8dd69489abc714de8e5872cdd0a39c580d"
        repo_ctx.download_and_extract(
            url = url,
            sha256 = sha256,
            type = "tar.gz",
        )
    else:
        fail("Operating system is NOT supported", attr = os_result)

castxml_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
