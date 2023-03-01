# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def petsc_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "petsc/petsc",
        commit = "v3.18.4",
        sha256 = "8da1da9588bc97cd162f76ef92ceeca4b188ba3b8842aa2f7a59043015021785",  # noqa
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
