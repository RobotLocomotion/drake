"""
Utility module for helping discover which vtk library produces which headers.

For creating repository.bzl, we seek a listing of the relevant header files a
given vtk library produces.  This module enumerates all possible header files
that a given vtk library produces.  Since the VTK build generates header files
the prerequisite for running this file is that you have:

1. An extracted (or cloned) VTK source tree.
2. A CMake configured build directory of (1).  You do not need to compile or
   install this build tree, but the CMake configure step will produce
   additional header files for a given VTK module.  You should use the same
   CMake arguments as specified in /tools/workspace/vtk/image/vtk-args.
3. A drake vtk-*.tar.gz tarball, as produced by ``build_binaries_with_docker``.
   This will be parsed as the "install reference", not all VTK headers get
   installed and we only want to enumerate headers that can actually be
   consumed.  VTK installs to a flat tree, meaning the install tree does not
   provide us with the ability to recover which VTK module a given header file
   came from.

The final argument to the script is the name of the VTK module to parse, e.g.,
``vtkCommonCore``.  This module will then compare all headers found in (1) and
(2) with headers found in (3).  There is a --debug option enabled to enumerate
everything that was found and skipped, but it is disabled by default to enable
piping this script to e.g., ``xclip -i -selection clipboard`` to paste into
``repository.bzl``.
"""
import argparse
from pathlib import Path
import tarfile
import sys

this_file_dir = Path(__file__).parent.absolute()

VTK_MODULES = [
    "vtkCommonColor",
    "vtkCommonComputationalGeometry",
    "vtkCommonCore",
    "vtkCommonDataModel",
    "vtkCommonExecutionModel",
    "vtkCommonMath",
    "vtkCommonMisc",
    "vtkCommonSystem",
    "vtkCommonTransforms",
    "vtkDICOMParser",
    "vtkFiltersCore",
    "vtkFiltersGeometry",
    "vtkFiltersGeneral",
    "vtkFiltersHybrid",
    "vtkFiltersSources",
    "vtkImagingCore",
    "vtkImagingMath",
    "vtkImagingSources",
    "vtkIOCore",
    "vtkIOExport",
    "vtkIOGeometry",
    "vtkIOImage",
    "vtkIOImport",
    "vtkIOLegacy",
    "vtkIOXML",
    "vtkIOXMLParser",
    "vtkRenderingCore",
    "vtkRenderingContext2D",
    "vtkRenderingFreeType",
    "vtkRenderingHyperTreeGrid",
    "vtkRenderingOpenGL2",
    "vtkRenderingSceneGraph",
    "vtkRenderingUI",
    "vtkRenderingVtkJS",
]
"""
A non-exhaustive list of vtk modules under consideration.

Each element must also be entered as a key in VTK_SOURCE_MAP below.
"""

VTK_SOURCE_MAP = {
    "vtkCommonColor": Path("Common") / "Color",
    "vtkCommonComputationalGeometry": Path("Common") / "ComputationalGeometry",
    "vtkCommonCore": Path("Common") / "Core",
    "vtkCommonDataModel": Path("Common") / "DataModel",
    "vtkCommonExecutionModel": Path("Common") / "ExecutionModel",
    "vtkCommonMath": Path("Common") / "Math",
    "vtkCommonMisc": Path("Common") / "Misc",
    "vtkCommonSystem": Path("Common") / "System",
    "vtkCommonTransforms": Path("Common") / "Transforms",
    "vtkDICOMParser": Path("Utilities") / "DICOMParser",
    "vtkFiltersCore": Path("Filters") / "Core",
    "vtkFiltersGeometry": Path("Filters") / "Geometry",
    "vtkFiltersGeneral": Path("Filters") / "General",
    "vtkFiltersHybrid": Path("Filters") / "Hybrid",
    "vtkFiltersSources": Path("Filters") / "Sources",
    "vtkImagingCore": Path("Imaging") / "Core",
    "vtkImagingMath": Path("Imaging") / "Math",
    "vtkImagingSources": Path("Imaging") / "Sources",
    "vtkIOCore": Path("IO") / "Core",
    "vtkIOExport": Path("IO") / "Export",
    "vtkIOGeometry": Path("IO") / "Geometry",
    "vtkIOImage": Path("IO") / "Image",
    "vtkIOImport": Path("IO") / "Import",
    "vtkIOLegacy": Path("IO") / "Legacy",
    "vtkIOXML": Path("IO") / "XML",
    "vtkIOXMLParser": Path("IO") / "XMLParser",
    "vtkRenderingCore": Path("Rendering") / "Core",
    "vtkRenderingContext2D": Path("Rendering") / "Context2D",
    "vtkRenderingFreeType": Path("Rendering") / "FreeType",
    "vtkRenderingHyperTreeGrid": Path("Rendering") / "HyperTreeGrid",
    "vtkRenderingOpenGL2": Path("Rendering") / "OpenGL2",
    "vtkRenderingSceneGraph": Path("Rendering") / "SceneGraph",
    "vtkRenderingUI": Path("Rendering") / "UI",
    "vtkRenderingVtkJS": Path("Rendering") / "VtkJS",
}
"""
Map of vtk library name to path under the source tree that the library lives.
"""


def main():
    parser = argparse.ArgumentParser(
        description="Find header files for a given VTK module.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    parser.add_argument(
        "-s",
        "--source-tree",
        dest="source_tree",
        type=str,
        help="The VTK source tree root (contains top-level CMakeLists.txt).",
        required=True,
    )
    parser.add_argument(
        "-b",
        "--build-tree",
        dest="build_tree",
        type=str,
        help="The VTK source build tree (where you configured CMake).",
        required=True,
    )
    parser.add_argument(
        "-i",
        "--install-tarball",
        dest="install_tarball",
        type=str,
        help="VTK install tarball to parse.",
        required=True,
    )
    parser.add_argument("--debug", action="store_true")
    # TODO(svenevs): add ability to dump all headers for all modules to find a
    # header when the module name is not known.
    parser.add_argument(
        "module", type=str, help="VTK module to parse.", choices=VTK_MODULES
    )

    args = parser.parse_args()

    module = args.module

    source_root = Path(args.source_tree).absolute()
    if not source_root.is_dir():
        sys.stderr.write(
            f"Provided source tree '{str(source_root)}' is not a directory!\n"
        )
        sys.exit(1)
    if (
        not (source_root / "CMakeLists.txt").is_file()
        or not (source_root / "Common" / "Core").is_dir()
    ):
        sys.stderr.write(
            f"Source tree '{str(source_root)}' did not have top-level "
            "CMakeLists.txt, or sub-directory 'Common/Core'.  Is this a VTK "
            "source tree?\n"
        )
        sys.exit(1)

    build_root = Path(args.build_tree).absolute()
    if not build_root.is_dir():
        sys.stderr.write(
            f"Build folder '{str(build_root)}' is not a directory!\n"
        )
        sys.exit(1)
    if not (build_root / "CMakeCache.txt").is_file():
        sys.stderr.write(
            f"Did not find CMakeCache.txt in '{str(build_root)}', did you "
            "already configure the VTK build using CMake?\n"
        )
        sys.exit(1)

    install_tarball = Path(args.install_tarball).absolute()
    if not install_tarball.is_file() or not install_tarball.name.endswith(
        ".tar.gz"
    ):
        sys.stderr.write(
            f"Install tarball '{str(install_tarball)}' must be a file with "
            "`.tar.gz` suffix!\n"
        )
        sys.exit(1)

    # First gather all of the installed files from the tarball.
    installed_headers = []
    HEADER_EXTS = [".h", ".hpp", ".hxx", ".txx"]
    try:
        with tarfile.open(install_tarball, "r") as vtk_tar_f:
            for name in vtk_tar_f.getnames():
                # NOTE: this does give some false positive results from e.g.,
                # kwwiml and other utility libraries, but we can ignore them
                # since we do an intersection below.
                if name.startswith("include"):
                    name_p = Path(name)
                    if (
                        name_p.name.startswith("vtk")
                        and name_p.suffix in HEADER_EXTS
                    ):
                        installed_headers.append(name_p.name)
    except Exception as e:
        sys.stderr.write(f"Error parsing '{str(install_tarball)}': {e}\n")
        sys.exit(1)

    # Gather all headers available in the source and build trees.
    source_module = source_root / VTK_SOURCE_MAP[module]
    sources = []
    mod = VTK_SOURCE_MAP[module]
    for ext in HEADER_EXTS:
        for tree in [source_root / mod, build_root / mod]:
            tree_glob = tree.glob(f"**/*{ext}")
            for tg in tree_glob:
                sources.append(tg.name)
    sources = list(set(sources))
    sources.sort()

    # End goal is the intersection of installed and source tree headers.
    found = list(set(installed_headers) & set(sources))
    found.sort()

    # NOTE: depending on CMake configure, you may get many more skipped headers
    # than if you were to run this within the docker build somehow.  This can
    # generally be ignored, the out-of-docker build will be a superset.
    if args.debug:
        vsep = "*" * 88
        print(f"{vsep}\n* Sources found:\n{vsep}")
        for s in sources:
            print(s)

        skipped = list(set(sources) - set(found))
        skipped.sort()

        print(f"\n{vsep}\n* Skipped:\n{vsep}")
        if skipped:
            for s in skipped:
                print(s)
        else:
            print("NONE")

        print(f"\n{vsep}\n* Install found:\n{vsep}")

    for f in found:
        print(f)


if __name__ == "__main__":
    main()
