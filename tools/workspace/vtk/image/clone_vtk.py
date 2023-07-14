"""Helper module to clone the VTK source tree.

This is kept in a separate file so that it can be added to the docker image and
run directly so that cloning can be cached in an earlier layer.
"""
import subprocess
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).parent.resolve()))
from vtk_common import vtk_git_ref, vtk_package_tree  # noqa: E402


def clone_vtk(vtk_git_ref: str, dest_dir: Path):
    """
    Clone VTK to dest_dir and checkout the specified vtk_git_ref.
    """
    # NOTE: a full clone is required, VTK_GIT_REF can be a commit, branch, tag,
    # or anything between.  Since cloning VTK can take a while, display the
    # progress meter.
    subprocess.check_call(
        [
            "git",
            "clone",
            "--progress",
            "--verbose",
            "https://gitlab.kitware.com/vtk/vtk.git",
            dest_dir.name,
        ],
        cwd=dest_dir.parent,
    )
    subprocess.check_call(
        [
            "git",
            "checkout",
            vtk_git_ref,
        ],
        cwd=dest_dir,
    )


if __name__ == "__main__":
    package_tree = vtk_package_tree()
    package_tree.root.mkdir(parents=True, exist_ok=True)
    clone_vtk(vtk_git_ref(), package_tree.source_dir)
