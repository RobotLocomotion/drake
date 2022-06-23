# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def petsc_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "petsc/petsc",
        commit = "v3.17.1",
        sha256 = "6dfe03613ccf5cd2d19f6267057ebb734a49e81e300f11ba7fc1601bbe585796",  # noqa
        build_file = "@drake//tools/workspace/petsc:package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            # Cherry-picked from upstream (to be removed once we upgrade).
            "@drake//tools/workspace/petsc:patches/baij.patch",
            # Cherry-picked from upstream (to be removed once we upgrade).
            "@drake//tools/workspace/petsc:patches/mal.patch",
            # Patch to fix dangerous global state in PETSc.
            "@drake//tools/workspace/petsc:patches/destroy.patch",
            "@drake//tools/workspace/petsc:patches/dlregispetsc.patch",
            "@drake//tools/workspace/petsc:patches/inherit.patch",
            "@drake//tools/workspace/petsc:patches/matrix.patch",
            "@drake//tools/workspace/petsc:patches/mpi.patch",
            "@drake//tools/workspace/petsc:patches/petscimpl.patch",
            "@drake//tools/workspace/petsc:patches/pname.patch",
            "@drake//tools/workspace/petsc:patches/remove_packages.patch",
            "@drake//tools/workspace/petsc:patches/tagm.patch",
        ],
    )
