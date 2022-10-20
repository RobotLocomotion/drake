# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gazebosim/sdformat",
        commit = "sdformat13_13.1.0",
        build_file = ":package.BUILD.bazel",
        sha256 = "4b40bf64bc7d3f22c89a2395c2802d5315a3fd5e5dbaf29b087c5c806180a9db",  # noqa
        patches = [
            ":patches/console.patch",
            ":patches/deprecation_unit_testing.patch",
            ":patches/no_global_config.patch",
            ":patches/no_urdf.patch",
        ],
        mirrors = mirrors,
    )
