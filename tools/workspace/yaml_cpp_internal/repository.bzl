load("//tools/workspace:github.bzl", "github_archive")

def yaml_cpp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "jbeder/yaml-cpp",
        commit = "yaml-cpp-0.9.0",
        sha256 = "25cb043240f828a8c51beb830569634bc7ac603978e0f69d6b63558dadefd49a",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/pr1398.patch",
            ":patches/emit-local-tag.patch",
        ],
        mirrors = mirrors,
    )
