load("//tools/workspace:github.bzl", "github_archive")

def msgpack_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "msgpack/msgpack-c",
        commit = "cpp-6.1.1",
        sha256 = "d7b119f292365d41403b41b40c2fefd82ebd81241e3c658fafe0e638fa54604a",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
