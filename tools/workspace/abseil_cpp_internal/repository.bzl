load("//tools/workspace:github.bzl", "github_archive")

def abseil_cpp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "abseil/abseil-cpp",
        commit = "79bdf3b41f01fe488dd2d82a2c6260cfbca544c4",
        sha256 = "65917799606c65a0881df2512f7967278f54576ba354a6ec9b0c30d3416448fc",  # noqa
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
