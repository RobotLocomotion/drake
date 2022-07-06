# -*- python -*-
load("@drake//tools/workspace:github.bzl", "github_archive")

def abseil_cpp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "abseil/abseil-cpp",
        commit = "93ad4284ac12077b7bac07a4743df1c564e7c957",
        sha256 = "00b436d3822754ed5c62039027705a61dc79cb947577ddc199996aa0df602dd4",  # noqa
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
