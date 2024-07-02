load("//tools/workspace/mosek:repository_impl.bzl", "repository_impl")

def mosek_repository(
        name,
        mirrors = None):
    repository_impl(
        name = name,
        # When the version is updated:
        # - tools/dynamic_analysis/tsan.supp may also need updating
        # - LICENSE.third_party may also need updating to match
        #   https://docs.mosek.com/latest/licensing/license-agreement-info.html
        version = "10.1.21",
        sha256 = {
            "mosektoolslinuxaarch64.tar.bz2": "c2d15979dc1190ff83949b2e79244137ed4013fdb47de90fb62836088749e0ca",  # noqa
            "mosektoolslinux64x86.tar.bz2": "f37b7b3806e467c64a02e95b2ab009f6fe8430f25ffc72ed56885f7684dec486",  # noqa
            "mosektoolsosxaarch64.tar.bz2": "f6e862cab171b7897a6f1ad21c3c0fbdf33dc1310f50c792295ab008321950c7",  # noqa
            "mosektoolsosx64x86.tar.bz2": "3ad45f7e535b6d3bb8be955f403ded30a7f186424057f11024afc57427cbb012",  # noqa
        },
        mirrors = mirrors,
    )
