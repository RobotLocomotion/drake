#!/usr/bin/env python3

import argparse
from glob import glob
from pathlib import Path

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
    "vtkRenderingOpenGL2",
    "vtkRenderingSceneGraph",
    "vtkRenderingUI",
    "vtkRenderingVtkJS",
]

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
    "vtkRenderingOpenGL2": Path("Rendering") / "OpenGL2",
    "vtkRenderingSceneGraph": Path("Rendering") / "SceneGraph",
    "vtkRenderingUI": Path("Rendering") / "UI",
    "vtkRenderingVtkJS": Path("Rendering") / "VtkJS",
}

if __name__ == "__main__":
    import tarfile
    from pathlib import Path

    headers = []
    modules = []
    with tarfile.open("/tmp/VTK-9.1.0.tar.gz", "r") as vtk_src_f:
        for name in vtk_src_f.getnames():
            p = Path(name)
            # Skip over directories we do not need to care about.
            if len(p.parts) < 2 or p.parts[1] in {"Examples",
                                                  "Testing",
                                                  "ThirdParty",
                                                  "Utilities",
                                                  "Views",
                                                  "Web",
                                                  "Wrapping"}:
                continue

            if p.parts[-1] == "vtk.module":
                modules.append(p)
            elif p.suffix in [".h", ".hpp", ".hxx", ".txx"]:
                headers.append(p)

        parents = [str(m.parent) for m in modules]
        vtk_src_file_map = {p: [] for p in parents}
        for h in headers:
            for p in parents:
                if str(h.parent) == p:
                    vtk_src_file_map[p].append(h.name)

        del_keys = []
        for p, hdr_list in vtk_src_file_map.items():
            if len(hdr_list) == 0:
                del_keys.append(p)
        for d in del_keys:
            del vtk_src_file_map[d]

        modules = []  # Only parse the ones we care about.
        vtk_src_module_map = {}
        for key in vtk_src_file_map:
            m_path = Path(key) / "vtk.module"
            member = vtk_src_f.getmember(str(m_path))
            contents = vtk_src_f.extractfile(member).read().decode("utf-8")
            seen_depends = False
            seen_priv_depends = False
            import re
            deps = []
            import pdb
            pdb.set_trace()
            for line in contents.splitlines():
                line_s = line.strip()
                if line_s == "DEPENDS":
                    seen_depends = True
                    continue
                elif line_s == "PRIVATE_DEPENDS":
                    seen_priv_depends = True
                    continue
                print(line)

        import json
        print(json.dumps(vtk_src_module_map, indent=2))

    import sys
    sys.exit(0)





    parser = argparse.ArgumentParser(description="Help find headers for bazel.")

    parser.add_argument("-s", "--source-tree", dest="source_tree", type=str,
        help="Root folder of VTK source tree.",
        default="/home/local/KHQ/stephen.mcdowell/Desktop/tri/vtk-91/VTK-9.1.0")
    parser.add_argument("-i", "--install-tree", dest="install_tree", type=str,
        help="Root folder of VTK install / tar extract.",
        default="/home/local/KHQ/stephen.mcdowell/Desktop/tri/drake-vtk/tools/workspace/vtk/bku/x/vtk-9")
    parser.add_argument("module", type=str, help="VTK Module to parse.",
        choices=VTK_MODULES)

    args = parser.parse_args()

    # i'm being wreckless, this script is just for me.
    module = args.module
    source_root = Path(args.source_tree)
    install_root = Path(args.install_tree)
    install_include = install_root / "include" / "vtk-9.1"

    if module == "ALL":
        all_installed_headers = []
        for suffix in [".h", ".hpp", ".hxx", ".txx"]:
            all_installed_headers += install_include.glob(f"**/*{suffix}")
        all_installed_headers = [p.name for p in all_installed_headers]
        all_installed_headers = list(set(all_installed_headers))
        all_installed_headers.sort()

        all_source_headers = []
        for suffix in [".h", ".hpp", ".hxx", ".txx"]:
            all_source_headers += source_root.glob(f"**/*{suffix}")
        all_source_headers = [p.name for p in all_source_headers]
        all_source_headers = list(set(all_source_headers))
        all_source_headers.sort()

        extra_installed = []
        for ih in all_installed_headers:
            if ih not in all_source_headers:
                extra_installed.append(ih)
        extra_installed = list(set(extra_installed))
        extra_installed.sort()
        print(extra_installed)
        print(len(extra_installed))

    else:
        # vtk has:
        #   cc
        #   h
        #   hpp
        #   hxx
        #   octree
        #   txx
        extensions = ["h", "hpp", "hxx", "txx"]

        source_module = source_root / VTK_SOURCE_MAP[module]
        sources = []
        mod = VTK_SOURCE_MAP[module]
        for ext in extensions:
            for tree in [source_root / mod, source_root / "build" / mod]:
                tree_glob = tree.glob(f"**/*.{ext}")
                for tg in tree_glob:
                    sources.append(tg.name)
        sources = list(set(sources))
        sources.sort()

        vsep = "*" * 88
        print(f"{vsep}\n* Sources found:\n{vsep}")
        for s in sources:
            print(s)

        installed = []
        for ext in extensions:
            inst_glob = install_include.glob(f"**/*.{ext}")
            for ig in inst_glob:
                installed.append(ig.name)

        installed = list(set(installed))
        installed.sort()

        found = list(set(installed) & set(sources))
        found.sort()

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





