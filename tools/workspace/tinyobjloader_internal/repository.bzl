load("//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "tinyobjloader/tinyobjloader",
        commit = "ee45fb41db95bf9563f2a41bc63adfa18475c2ee",
        sha256 = "10347059b35739d13b37ea40febc38dfe1c482e623026ee35771ab08c3e03883",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            ":patches/faster_float_parsing.patch",
            ":patches/default_texture_color.patch",
        ],
    )
