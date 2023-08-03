from clone_vtk import clone_vtk

from vtk_cmake_configure_args import vtk_cmake_configure_args

from vtk_common import (
    build_vtk,
    package_vtk,
    system_is_linux,
    vtk_git_ref,
    vtk_package_tree,
)


def main() -> None:
    package_tree = vtk_package_tree()
    package_tree.root.mkdir(parents=True, exist_ok=True)
    # On Linux the Dockerfile should have already staged our clone, assert that
    # it exists.  On macOS, perform the clone only if the folder does not
    # already exist (it can take some time to fully clone VTK).
    if system_is_linux():
        assert (
            package_tree.source_dir.is_dir()
        ), f"Expected {package_tree.source_dir} to be a directory."
    else:
        clone_vtk(vtk_git_ref(), package_tree.source_dir)
    build_vtk(package_tree, vtk_cmake_configure_args())
    package_vtk(package_tree)


if __name__ == "__main__":
    main()
