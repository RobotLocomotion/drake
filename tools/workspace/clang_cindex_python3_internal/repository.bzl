load("//tools/workspace:github.bzl", "github_archive")

def clang_cindex_python3_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "wjakob/clang-cindex-python3",
        commit = "b0b92c9395f3927af6a96ac8915e700259d2f55b",
        sha256 = "a09d12a4303dffe9f53e62149d829e651d79e6848a712fab1d77bed382efc37b",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
        patch_cmds = ["mkdir clang && mv *.py clang"],
        patches = [
            ":patches/clang-15.patch",
        ],
    )
