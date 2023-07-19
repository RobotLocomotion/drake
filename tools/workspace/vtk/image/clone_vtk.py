"""Helper module to clone the VTK source tree.

On Linux (from docker) this file is run directly in an early layer so that it
can be cached.  This is the only reason this method is in its own file (it does
not update often => the layer stays cached).  On macOS, ``clone_vtk`` is
imported and run directly.
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
    # NOTE: vtk_git_ref can be a commit, branch, tag, or anything between.
    # Since cloning VTK can take a while, display the progress meter.
    subprocess.check_call(
        [
            "git",
            "clone",
            "--filter=blob:none",
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


def main():
    package_tree = vtk_package_tree()
    package_tree.root.mkdir(parents=True, exist_ok=True)
    clone_vtk(vtk_git_ref(), package_tree.source_dir)


if __name__ == "__main__":
    main()
