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
        # Nightly: https://data.kitware.com/#collection/57b5c9e58d777f126827f5a1/folder/5f873c1350a41e3d19eae63e # noqa
        url = "https://data.kitware.com/api/v1/file/6006eda52fa25629b9fca696/download"  # noqa
        sha256 = "170a7b2a4e1a8ed1745e5238413ef6946a5d1665959e653b46266ac6b4a55958"  # noqa
        repo_ctx.download_and_extract(
            url = url,
            sha256 = sha256,
            type = "tar.gz",
        )
    elif os_result.is_ubuntu:
        # Nightly: https://data.kitware.com/#collection/57b5c9e58d777f126827f5a1/folder/5f873c1350a41e3d19eae63e # noqa
        url = "https://data.kitware.com/api/v1/file/6006edd02fa25629b9fca6a6/download"  # noqa
        sha256 = ""  # noqa
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
