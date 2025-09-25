load("//tools/workspace:github.bzl", "github_archive")

def msgpack_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "msgpack/msgpack-c",
        commit = "cpp-7.0.0",
        sha256 = "070881ebea9208cf7e731fd5a46a11404025b2f260ab9527e32dfcb7c689fbfc",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/type_traits_remove_volatile.patch",
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
