load("//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        local_repository_override = "/home/seancurtis/code/tinyobjloader",
        repository = "tinyobjloader/tinyobjloader",
        commit = "cab4ad7254cbf7eaaafdb73d272f99e92f166df8",
        sha256 = "4cc086e2d6c7d521231745c51fed8ac2467a0570a544ff4c9e96023f2512cfb2",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            ":patches/faster_float_parsing.patch",
            ":patches/default_texture_color.patch",
            ":patches/non_copy_parse_from_string.patch",
        ],
    )
