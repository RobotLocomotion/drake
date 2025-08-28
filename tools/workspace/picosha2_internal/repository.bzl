load("//tools/workspace:github.bzl", "github_archive")

def picosha2_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "okdshin/PicoSHA2",
        commit = "161cb3fc4170fa7a3eca9e582cebd27cc4d1fe29",
        sha256 = "6cf473a00c98298d3ddee0aed853e3c799791f49dbc01c996ea46cb248e85802",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
