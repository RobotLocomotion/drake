load("//tools/workspace:github.bzl", "github_archive")

def yaml_cpp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "jbeder/yaml-cpp",
        commit = "0.8.0",
        sha256 = "fbe74bbdcee21d656715688706da3c8becfd946d92cd44705cc6098bb23b3a16",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/pr1303.patch",
            ":patches/upstream/pr1336.patch",
            ":patches/upstream/b64_decode_failure_is_empty.patch",
            ":patches/emit-local-tag.patch",
        ],
        mirrors = mirrors,
    )
