# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def clang_cindex_python3_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "wjakob/clang-cindex-python3",
        commit = "6a00cbc4a9b8e68b71caf7f774b3f9c753ae84d5",
        sha256 = "828e0d6238e2129a9e08071750dc16ba10e38eacf96f21b8a71e501c2085b282",
        build_file = "@drake//tools/workspace/clang_cindex_python3:package.BUILD.bazel",
        mirrors = mirrors,
        patch_cmds = ["mkdir clang && mv *.py clang"]
    )
