load("//tools/workspace:github.bzl", "github_archive")

def spral_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ralna/spral",
        commit = "v2025.09.18",
        sha256 = "1358168ac95297049e4fc810e54a16e0c765796cfbaa156e09979e2620b7dae7",  # noqa
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
