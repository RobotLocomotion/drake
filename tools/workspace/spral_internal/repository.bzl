load("//tools/workspace:github.bzl", "github_archive")

def spral_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ralna/spral",
        commit = "v2024.05.08",
        sha256 = "0795c10c1c4dab1cf8c2de4024296d75d9d83b7525e82c77584c16060e29e4f5",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/pr222.patch",
            ":patches/drake_vendor.patch",
            ":patches/no_fortran_profiling.patch",
        ],
        patch_cmds = [
            # These sed commands switch the C++ code to nominally use hidden
            # symbols. Ideally we would use our vendor_cxx tool instead, but
            # its parser cannot digest spral's flavor of C++ code (yet).
            "sed -i -e 's|^namespace spral |namespace spral __attribute__((visibility(\"hidden\"))) |;' $(find src -name *.hxx -o -name *.cxx)",  # noqa
            "sed -i -e 's|;|__attribute__((visibility(\"hidden\")));|;' src/ssids/cpu/kernels/wrappers.hxx",  # noqa
        ],
        mirrors = mirrors,
    )
