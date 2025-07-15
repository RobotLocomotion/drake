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
        version = "11.0.24",
        sha256 = {
            "mosektoolslinuxaarch64.tar.bz2": "68e94abb10087bf38dd4b378cc51a412dc1f94171ef50dff9075a4c1bf909bb6",  # noqa
            "mosektoolslinux64x86.tar.bz2": "f058e4bec5cde899cf5193f7ac966a923a256c335466a0678604068a51404362",  # noqa
            "mosektoolsosxaarch64.tar.bz2": "b0bd9232e45597d098b9464eb6302b17cf0f3bd5679e06bce1e74c046604da34",  # noqa
        },
        mirrors = mirrors,
    )
