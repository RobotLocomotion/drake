load("//tools/workspace:github.bzl", "github_archive")

def abseil_cpp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "abseil/abseil-cpp",
        commit = "643b48a3b4362a932c0e41afce62deb55adf825b",
        sha256 = "8ce1c7335768192d0bafad19f24e17c6ebcb18f1968db9a38dc325a15af2ef66",  # noqa
        patches = [
            ":patches/disable_int128_on_clang.patch",
            ":patches/hidden_visibility.patch",
            ":patches/inline_namespace.patch",
        ],
        patch_cmds = [
            # Force linkstatic = 1 everywhere. First, remove the few existing
            # uses so that we don't get "duplicate kwarg" errors. Then, add it
            # anywhere that linkopts already appears.
            "sed -i -e 's|linkstatic = 1,||; s|linkopts = |linkstatic = 1, linkopts =|' absl/*/BUILD.bazel",  # noqa
        ],
        mirrors = mirrors,
    )
