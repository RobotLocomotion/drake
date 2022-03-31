# -*- python -*-
load("@drake//tools/workspace:github.bzl", "github_archive")

def abseil_cpp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "abseil/abseil-cpp",
        commit = "3204cc0625230e9876f0310a6dea0014210ab325",
        sha256 = "1e60f834f9590197b26addf5060381163a02fe25898978727b4ad4eae1b875f9",  # noqa
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
