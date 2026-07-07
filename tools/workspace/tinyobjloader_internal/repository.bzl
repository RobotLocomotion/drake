load("//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "tinyobjloader/tinyobjloader",
        upgrade_type = "commit",
        commit = "45636bdcef1a4fec140346b90c0b50bf0bc3e23b",
        sha256 = "7a4ae25cfd6ba776004cac717fe7e39b997974bf05167ca2468d8b31fa257fc9",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            ":patches/upstream/in_memory_parsing_support.patch",
            ":patches/faster_float_parsing.patch",
            ":patches/default_texture_color.patch",
            ":patches/silence_materials.patch",
            ":patches/remove_msvc_intrin.patch",
        ],
    )
