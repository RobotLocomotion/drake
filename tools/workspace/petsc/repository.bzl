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
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            # Cherry-picked from upstream (to be removed once
            # https://gitlab.com/petsc/petsc/-/merge_requests/5228/commits
            # is included in the release).
            ":patches/baij.patch",
            # Patch to fix dangerous global state in PETSc.
            ":patches/destroy.patch",
            ":patches/dlregispetsc.patch",
            ":patches/inherit.patch",
            ":patches/matrix.patch",
            ":patches/mpi.patch",
            ":patches/petscimpl.patch",
            ":patches/pname.patch",
            ":patches/remove_packages.patch",
            ":patches/tagm.patch",
        ],
    )
