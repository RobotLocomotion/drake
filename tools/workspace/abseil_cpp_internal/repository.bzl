# -*- python -*-
load("@drake//tools/workspace:github.bzl", "github_archive")

def abseil_cpp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "abseil/abseil-cpp",
        commit = "ef799d337201e02688c6e07b0cac8912b86f505d",
        sha256 = "9f841741770fd773c8109f23fed60ec1e758ce12a52fbb298eeaf891b1dbb1cd",  # noqa
        patches = [
            "@drake//tools/workspace/abseil_cpp_internal:patches/disable_int128_on_clang.patch",  # noqa
            "@drake//tools/workspace/abseil_cpp_internal:patches/fix_constexpr_storage_deprecation.patch",  # noqa
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
