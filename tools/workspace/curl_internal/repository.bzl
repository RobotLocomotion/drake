load("//tools/workspace:github.bzl", "github_archive")

def curl_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "curl/curl",
        upgrade_advice = """
        In case of a cmake_configure_file build error when upgrading curl,
        update cmakedefines.bzl to match the new upstream definitions.
        """,
        commit = "curl-8_13_0",
        sha256 = "ccc5ba45d9f5320c70ffb24e5411b66ba55ea1f333bf78be0963ed90a9328699",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/49701094fcc8fc510af97e439ad46415342d367f.patch",
            ":patches/upstream/8f496d05b681f112225c839c8907b83aeebbf89b.patch",
        ],
        mirrors = mirrors,
    )
