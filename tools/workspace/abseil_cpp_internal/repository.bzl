load("//tools/workspace:github.bzl", "github_archive")

def abseil_cpp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "abseil/abseil-cpp",
        commit = "20a1220136952ff620a47a5d6b94243e002c74f3",
        sha256 = "6bec3db7ba0c511614ccae38e803a31db908712560ad170c1a62309499e2c264",  # noqa
        patches = [
            ":patches/upstream/specific_iostream_includes.patch",
            ":patches/disable_int128_on_clang.patch",
            ":patches/hidden_visibility.patch",
            ":patches/inline_namespace.patch",
        ],
        patch_cmds = [
            # Force linkstatic = 1 everywhere. First, remove the few existing
            # uses so that we don't get "duplicate kwarg" errors. Then, add it
            # anywhere that linkopts already appears.
            "sed -i -e 's|linkstatic = 1,||; s|linkopts = |linkstatic = 1, linkopts = |' $(find absl -name BUILD.bazel)",  # noqa
        ],
        mirrors = mirrors,
    )
