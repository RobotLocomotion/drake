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
        # - LICENSE.third_party may also need updating to match
        #   https://docs.mosek.com/latest/licensing/license-agreement-info.html
        version = "12.0.1",
        sha256 = {
            "mosektoolslinuxaarch64.tar.bz2": "6b552c24f81562ff07fda31af65d85c431873c453df8bad79373957df8baddd7",  # noqa
            "mosektoolslinux64x86.tar.bz2": "b600e25146e5e45dc10842ccc01c2d0a6ada8b801e8166e3f116189049b471e4",  # noqa
            "mosektoolsosxaarch64.tar.bz2": "c6c86d11aab55ca035caccd64c3aec4aec4140212e4dd8a32aacd5d66c1b83fa",  # noqa
        },
        mirrors = mirrors,
    )
