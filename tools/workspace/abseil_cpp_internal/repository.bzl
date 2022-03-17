# -*- python -*-
load("@drake//tools/workspace:github.bzl", "github_archive")

def abseil_cpp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "abseil/abseil-cpp",
        commit = "c5a424a2a21005660b182516eb7a079cd8021699",
        sha256 = "9bbe4fd382bbd2911f31e8d70eae035a2168f8ca00d5aa51586c5ac7c86010b3",  # noqa
        patches = [
            "@drake//tools/workspace/abseil_cpp_internal:patches/hidden_visibility.patch",  # noqa
            "@drake//tools/workspace/abseil_cpp_internal:patches/inline_namespace.patch",  # noqa
        ],
        patch_cmds = [
            # Force linkstatic = 1 everywhere. First, remove the few existing
            # uses so that we don't get "duplicate kwarg" errors. Then, add it
            # anywhere that linkopts already appears.
            "sed -i -e 's|linkstatic = 1,||; s|linkopts = |linkstatic = 1, linkopts =|' absl/*/BUILD.bazel",  # noqa
        ],
        mirrors = mirrors,
    )
