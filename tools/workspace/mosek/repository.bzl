load("//tools/workspace/mosek:repository_impl.bzl", "repository_impl")

def mosek_repository(
        name,
        mirrors = None):
    repository_impl(
        name = name,
        # When the version is updated:
        # - Documentation in solvers needs updating, e.g.:
        #     git grep -l 'https://docs.mosek.com/' | xargs -n1 sed -i \
        #     '/https:\/\/docs\.mosek\.com/s,/11\.0/capi,/11.1/capi,
        # - tools/wheel/image/setup.py must also be updated
        # - tools/dynamic_analysis/tsan.supp may also need updating
        # - LICENSE.third_party may also need updating to match
        #   https://docs.mosek.com/latest/licensing/license-agreement-info.html
        version = "11.1.2",
        sha256 = {
            "mosektoolslinuxaarch64.tar.bz2": "59bec3262d3654d6754a89ea0a787f039ac90ca37f2093d2c6fd883672350ec8",  # noqa
            "mosektoolslinux64x86.tar.bz2": "61416cc09c9354e6e90fb65a48c97dc4d86d5e0634ac0380a9c4e7ffbd73bf00",  # noqa
            "mosektoolsosxaarch64.tar.bz2": "e09457a0ff2a74f701f366474bd135ae7f4d3caf08ef32b5666f2b5c1ced4b97",  # noqa
        },
        mirrors = mirrors,
    )
