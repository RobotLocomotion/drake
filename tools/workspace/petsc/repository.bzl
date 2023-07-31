load("@drake//tools/workspace:github.bzl", "github_archive")

def petsc_repository(
        name,
        mirrors = None):
    """The @petsc external is deprecated in Drake's WORKSPACE and will be
    removed on or after 2023-11-01.
    """
    github_archive(
        name = name,
        repository = "petsc/petsc",
        commit = "v3.19.3",
        sha256 = "d9cf7b0c057e12b903cb27007adc98728aabf09c5accb0cc65112c756b8fb68a",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            # Patch to fix dangerous global state in PETSc.
            ":patches/destroy.patch",
            ":patches/dlregispetsc.patch",
            ":patches/inherit.patch",
            ":patches/matrix.patch",
            ":patches/mpi.patch",
            ":patches/petscimpl.patch",
            ":patches/petsc_creationidx_keyval.patch",
            ":patches/pname.patch",
            ":patches/remove_packages.patch",
            ":patches/tagm.patch",
        ],
    )
