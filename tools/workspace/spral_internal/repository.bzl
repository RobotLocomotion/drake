load("//tools/workspace:github.bzl", "github_archive")

def spral_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ralna/spral",
        commit = "v2025.03.06",
        sha256 = "1da8576a64a415166d12b529cd30977a1917d2867509f9cb6682aaab2a57b21c",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
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
