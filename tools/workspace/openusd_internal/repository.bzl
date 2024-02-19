load("//tools/workspace:github.bzl", "github_archive")

def openusd_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "PixarAnimationStudios/OpenUSD",
        commit = "v23.11",
        sha256 = "2add389b121568f3dfb9b7e4f4551a6c8445ae353f1725a0753c8ce5639c4f83",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/onetbb.patch",
        ],
        mirrors = mirrors,
    )
