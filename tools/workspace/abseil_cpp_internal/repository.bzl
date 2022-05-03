# -*- python -*-
load("@drake//tools/workspace:github.bzl", "github_archive")

def abseil_cpp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "abseil/abseil-cpp",
        commit = "cc04c0e07427deb78091231238189d14561a9441",
        sha256 = "8a8baac74906fcb81396e2ea7e51e4729ce04981483b1d77eef09909bccc8b29",  # noqa
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
