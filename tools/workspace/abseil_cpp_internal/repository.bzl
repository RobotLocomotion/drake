load("//tools/workspace:github.bzl", "github_archive")

def abseil_cpp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "abseil/abseil-cpp",
        commit = "3ee08f329825b3f68c988b212bffd332b58e4275",
        sha256 = "01474554fc2981e92fcda71c7ce29d3ed4ee8dd0707b346da3f8978cd349fa33",  # noqa
        patches = [
            ":patches/upstream/civil_time_linkopts.patch",
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
