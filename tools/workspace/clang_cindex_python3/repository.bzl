# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def clang_cindex_python3_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "wjakob/clang-cindex-python3",
        commit = "9dcf4f16757c9b6446910e4de51ed27ee962b81b",
        sha256 = "65c26ec7fe09c54479ce5f375ccd5dd11e4e8bb11e47681c254be0d9bcd79164",  # noqa
        build_file = "@drake//tools/workspace/clang_cindex_python3:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
        patch_cmds = ["mkdir clang && mv *.py clang"],
    )
