# -*- python -*-

load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def yaml_cpp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "jbeder/yaml-cpp",
        commit = "yaml-cpp-0.7.0",
        sha256 = "43e6a9fcb146ad871515f0d0873947e5d497a1c9c60c58cb102a97b47208b7c3",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":emit-local-tag.patch",
        ],
        mirrors = mirrors,
    )
