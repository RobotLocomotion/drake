load("//tools/workspace:github.bzl", "github_archive")

def abseil_cpp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "abseil/abseil-cpp",
        commit = "67d126083c1584dd7dc584d700f853afaec365ca",
        sha256 = "8652366395b2f20628281fd98c4413e9947d989fcb214f8bdc56351e8cd7e7d4",  # noqa
        patches = [
            ":patches/disable_int128_on_clang.patch",
            ":patches/hidden_visibility.patch",
            ":patches/inline_namespace.patch",
        ],
        patch_cmds = [
            # Back-fill a linkopts key missing in the upstream build system.
            "sed -i -e 's|name = .civil_time.,|name = \"civil_time\", linkopts = [],|' absl/time/internal/cctz/BUILD.bazel",  #noqa
            # Force linkstatic = 1 everywhere. First, remove the few existing
            # uses so that we don't get "duplicate kwarg" errors. Then, add it
            # anywhere that linkopts already appears.
            "sed -i -e 's|linkstatic = 1,||; s|linkopts = |linkstatic = 1, linkopts = |' $(find absl -name BUILD.bazel)",  # noqa
        ],
        mirrors = mirrors,
    )
