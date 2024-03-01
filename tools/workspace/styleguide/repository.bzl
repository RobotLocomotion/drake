load("//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # repository = "RobotLocomotion/styleguide",
        # commit = "6aae49e2b5861cd298d681d628830a6261b947c3",
        # sha256 = "4a46dedf35d61c0dfeeb4e3ce6c6593c59265624332d849268a787ca1cc95a2a",  # noqa
        # TODO(svenevs): after a styleguide PR lands, do a drake PR to just
        # update the styleguide and then rebase python 3.12 PR.
        repository = "svenevs/styleguide",
        commit = "0c13a6c739cd3835b102caf25c31d00713f4ff56",
        sha256 = "30fae571aa0dd01f5b3135ca5962466dc46bdad1fd18b3bc6479679a786d7605",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/sre_deprecation.patch",
            ":patches/test_paths.patch",
        ],
        mirrors = mirrors,
    )
