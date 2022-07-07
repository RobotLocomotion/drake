# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def petsc_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "petsc/petsc",
        commit = "v3.17.3",
        sha256 = "7ff5bc5e58057761f94004e79e5da73d3cd308699a9f244fe4406abdad7a521a",  # noqa
        build_file = "@drake//tools/workspace/petsc:package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            # Cherry-picked from upstream (to be removed once
            # https://gitlab.com/petsc/petsc/-/merge_requests/5228/commits
            # is included in the release).
            "@drake//tools/workspace/petsc:patches/baij.patch",
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
