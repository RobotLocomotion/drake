load("@drake//tools/workspace:github.bzl", "github_archive")

def abseil_cpp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "abseil/abseil-cpp",
        commit = "2927340217c37328319b5869285a6dcdbc13e7a7",
        sha256 = "4d77240cee3efe967a6445021c926d43f0853744e5c4e3d6af556ef3300b1c75",  # noqa
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
