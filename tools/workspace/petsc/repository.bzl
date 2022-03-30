# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def petsc_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "petsc/petsc",
        commit = "v3.16.5",
        sha256 = "1adf81bfb42104af469443fbcdb4211a7cae674dcb46b6c312ff29a7ffb8ff45",  # noqa
        build_file = "@drake//tools/workspace/petsc:package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            # Patch to fix dangerous global state in PETSc.
            "@drake//tools/workspace/petsc:patches/destroy.patch",
            "@drake//tools/workspace/petsc:patches/inherit.patch",
            "@drake//tools/workspace/petsc:patches/matrix.patch",
            "@drake//tools/workspace/petsc:patches/mpi.patch",
            "@drake//tools/workspace/petsc:patches/petscimpl.patch",
            "@drake//tools/workspace/petsc:patches/pname.patch",
            "@drake//tools/workspace/petsc:patches/remove_packages.patch",
            "@drake//tools/workspace/petsc:patches/tagm.patch",
        ],
    )
