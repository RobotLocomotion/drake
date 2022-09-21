# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def petsc_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "petsc/petsc",
        commit = "v3.17.4",
        sha256 = "413098359fc44033e4d63eef25b89b30db9439307f9f369e5405e3f1ed3aa9a0",  # noqa
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
