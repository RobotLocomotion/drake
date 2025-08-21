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
        commit = "curl-8_15_0",
        sha256 = "2937cadde007aa3a52a17c21ac9153ea054700f37926d1d96602bf07e888c847",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/base64_definition.patch",
        ],
        mirrors = mirrors,
    )
