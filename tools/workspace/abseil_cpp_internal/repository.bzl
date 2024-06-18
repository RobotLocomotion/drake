load("//tools/workspace:github.bzl", "github_archive")

def abseil_cpp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "abseil/abseil-cpp",
        commit = "ed34153e0d9cd15f9b9eb45e86b457e0a495aeea",
        sha256 = "a1537ce57c1392682cb675add67f147edc230103ecc7103b4dd26c4149b8bf88",  # noqa
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
