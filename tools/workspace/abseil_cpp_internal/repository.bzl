load("//tools/workspace:github.bzl", "github_archive")

def abseil_cpp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "abseil/abseil-cpp",
        upgrade_type = "release",
        commit = "20260526.0",
        sha256 = "6e1aee535473414164bf83e4ebc40240dec71a4701f8a642d906e95bea1aea0c",  # noqa
        patches = [
            ":patches/upstream/include_bmi2intrin.patch",
            ":patches/upstream/internal_c_symbol.patch",
            ":patches/hidden_visibility.patch",
            ":patches/inline_namespace.patch",
        ],
        patch_cmds = [
            "echo 'exports_files([\"drake_repository_metadata.json\"])' >> BUILD.bazel",  # noqa
            # Force linkstatic = 1 everywhere. First, remove the few existing
            # uses so that we don't get "duplicate kwarg" errors. Then, add it
            # anywhere that linkopts already appears.
            "sed -i -e 's|linkstatic = 1,||; s|linkopts = |linkstatic = 1, linkopts = |' $(find absl -name BUILD.bazel)",  # noqa
        ],
        mirrors = mirrors,
    )
