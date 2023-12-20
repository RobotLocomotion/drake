load("//tools/workspace:github.bzl", "github_archive")

def rnv_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "hartwork/rnv",
        commit = "e2435bfd9e67a1e8b3a34bae6919ee572465d435",
        sha256 = "17a3b36a47a7c2be176b6152754eb362b9103b6656fc00736230f99555df8a37",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/doc_api.patch",
            ":patches/memcheck.patch",
            ":patches/no_after_in_messages.patch",
            ":patches/no_exit.patch",
            ":patches/no_main_symbol.patch",
        ],
        mirrors = mirrors,
    )
