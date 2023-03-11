load("@drake//tools/workspace:github.bzl", "github_archive")

def nlopt_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "stevengj/nlopt",
        commit = "v2.7.1",
        sha256 = "db88232fa5cef0ff6e39943fc63ab6074208831dc0031cf1545f6ecd31ae2a1a",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/remove_luksan.patch",
            ":patches/vendoring.patch",
        ],
        mirrors = mirrors,
    )
