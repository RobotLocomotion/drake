# Enforce that the script is being sourced, the parent script will have a bash
# array `vtk_cmake_args` defined to use on the final CMake configure line.  To
# support developer oversight the arguments will be printed to the console.
#
# The four different build flavors (linux, linux_wheel, mac, mac_wheel) share
# many common arguments, but different (platform specific) arguments such as
# dependencies must change depending on the flavor.  This file serves as the
# single point of coordination between the build flavors.
if [[ "${0}" == "${BASH_SOURCE[0]}" ]]; then
    echo 'vtk-cmake-args.sh: this file must be sourced.' >&2
    exit 1
fi
if ! [[ "${drake_vtk_build_flavor}" =~ ^(linux|linux_wheel|mac|mac_wheel)$ ]]; then
    echo 'vtk-cmake-args.sh: `drake_vtk_build_flavor` must be one of ' >&2
    echo '`linux`, `mac`, `linux_wheel` or `mac_wheel`.' >&2
    exit 1
fi

# Define the C++ standard.
if [[ "${drake_vtk_build_flavor}" == "linux" ]]; then
    # C++ standard on Linux is defined by `tools/linux-${codename}.bazelrc`.
    if [[ "${codename}" == "focal" ]]; then
        readonly _vtk_cxx_std="17"
    elif [[ "${codename}" == "jammy" ]]; then
        readonly _vtk_cxx_std="20"
    else
        echo "Unsupported distrbution: ${codename}." >&2
        exit 1
    fi
elif [[ "${drake_vtk_build_flavor}" == "linux_wheel" ]]; then
    # TODO(svenevs): is this the right C++ standard choice for linux wheel?
    readonly _vtk_cxx_std="17"
elif [[ "${drake_vtk_build_flavor}" =~ ^(mac|mac_wheel)$ ]]; then
    # C++ standard on macOS is defined by `tools/macos.bazelrc`.
    readonly _vtk_cxx_std="20"
else
    echo 'Unknown build flavor, no known C++ standard.' >&2
    exit 1
fi

# For wheel builds we build a handful of external dependencies (in
# `tools/wheel/image/dependencies/projects`), as well as request that VTK bundle
# additional third party libraries that on native builds drake has a dependency
# on in `tools/workspace/{dependency}`.
if [[ "${drake_vtk_build_flavor}" =~ ^(linux_wheel|mac_wheel)$ ]]; then
    readonly _vtk_use_external="OFF"
    readonly _vtk_fontconfig="DONT_WANT"
else
    readonly _vtk_use_external="ON"
    readonly _vtk_fontconfig="DEFAULT"
fi

# TODO(svenevs): macOS and pugixml split.
if [[ "${drake_vtk_build_flavor}" =~ ^(mac|mac_wheel)$ ]]; then
    readonly _vtk_external_pugixml="ON"
else
    readonly _vtk_external_pugixml="OFF"
fi

# On macOS use the Cocoa graphics framework.
if [[ "${drake_vtk_build_flavor}" =~ ^(mac|mac_wheel)$ ]]; then
    readonly _vtk_use_cocoa="ON"
else
    readonly _vtk_use_cocoa="OFF"
fi

# Add fortification / hardening flags for redistributables.
# TODO(svenevs): fix the linking flags on macOS.
# TODO(svenevs): finish going through hardening-check on Linux.
vtk_cmake_args=(
    "-DCMAKE_C_FLAGS:STRING='-D_FORTIFY_SOURCE=2 -fstack-protector-strong -Wno-deprecated-declarations'"
    "-DCMAKE_CXX_FLAGS:STRING='-D_FORTIFY_SOURCE=2 -fstack-protector-strong -Wno-deprecated-declarations'"
)
if [[ "${codename}" != "mac" ]]; then
    # Not available / applicable on clang.
    vtk_cmake_args+=(
        "-DCMAKE_EXE_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro'"
        "-DCMAKE_MODULE_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro'"
        "-DCMAKE_SHARED_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro'"
    )
fi

# Initialize with the common CMake configuration arguments that are common
# across all of the different build flavors.
vtk_cmake_args+=(
    # NOTE: CMAKE_INSTALL_PREFIX, CMAKE_BUILD_TYPE, and generator choice are set
    # by the wrapping build scripts:
    # - linux: tools/workspace/vtk/image/build-and-package-vtk.sh
    # - mac: tools/workspace/vtk/build_mac_binary
    # - linux_wheel, mac_wheel: tools/wheel/image/build-vtk.sh
    # Basic build configuration.
    "-DBUILD_SHARED_LIBS:BOOL=OFF"
    "-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON"
    # Enforce an exact CMake C++ standard.
    "-DCMAKE_CXX_STANDARD:STRING=${_vtk_cxx_std}"
    "-DCMAKE_CXX_STANDARD_REQUIRED:BOOL=ON"
    "-DCMAKE_CXX_EXTENSIONS:BOOL=OFF"
    # VTK by default installs license files to `share/licenses/PROJECT_NAME`, we
    # want the license files to end up in `share/doc/PROJECT_NAME`.
    "-DCMAKE_INSTALL_LICENSEDIR:STRING=share/doc/vtk-9.2"
    # VTK-specific build configuration.
    "-DVTK_ABI_NAMESPACE_NAME:STRING='drake_vtk __attribute__ ((visibility (\"hidden\")))'"

    # TODO(svenevs): diagnose and fix this warning
    # [2379/2512] Building CXX object IO/Geometry/CMakeFiles/IOGeometry.dir/vtkGLTFWriter.cxx.o
    # In file included from /vtk/build/IO/Geometry/vtkIOGeometryModule.h:45,
    #                  from /vtk/src/IO/Geometry/vtkGLTFWriter.h:42,
    #                  from /vtk/src/IO/Geometry/vtkGLTFWriter.cxx:15:
    # /vtk/build/Common/Core/vtkABINamespace.h:6:98: warning: 'visibility' attribute is meaningless since members of the anonymo
    # us namespace get local symbols [-Wattributes]
    #     6 | #define VTK_ABI_NAMESPACE_BEGIN inline namespace drake_vtk __attribute__ ((visibility ("hidden"))) {
    #       |                                                                                                  ^
    # /vtk/src/IO/Geometry/vtkGLTFWriter.cxx:65:1: note: in expansion of macro 'VTK_ABI_NAMESPACE_BEGIN'
    #    65 | VTK_ABI_NAMESPACE_BEGIN

    "-DBUILD_TESTING:BOOL=OFF"
    "-DVTK_LEGACY_REMOVE:BOOL=ON"
    "-DVTK_ENABLE_WRAPPING:BOOL=OFF"
    "-DVTK_WRAP_PYTHON:BOOL=OFF"
    "-DVTK_USE_COCOA:BOOL=${_vtk_use_cocoa}"
    # VTK Modules to include / exclude.
    "-DVTK_BUILD_ALL_MODULES:BOOL=OFF"
    "-DVTK_GROUP_ENABLE_Imaging:STRING=YES"
    "-DVTK_GROUP_ENABLE_MPI:STRING=DONT_WANT"
    "-DVTK_GROUP_ENABLE_Qt:STRING=NO"
    "-DVTK_GROUP_ENABLE_Rendering:STRING=YES"
    "-DVTK_GROUP_ENABLE_StandAlone:STRING=DONT_WANT"
    "-DVTK_GROUP_ENABLE_Views:STRING=DONT_WANT"
    "-DVTK_GROUP_ENABLE_Web:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_AcceleratorsVTKmCore:STRING=DEFAULT"
    "-DVTK_MODULE_ENABLE_VTK_AcceleratorsVTKmDataModel:STRING=DEFAULT"
    "-DVTK_MODULE_ENABLE_VTK_AcceleratorsVTKmFilters:STRING=DEFAULT"
    "-DVTK_MODULE_ENABLE_VTK_ChartsCore:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_CommonArchive:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_CommonColor:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_CommonComputationalGeometry:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_CommonCore:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_CommonDataModel:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_CommonExecutionModel:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_CommonMath:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_CommonMisc:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_CommonSystem:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_CommonTransforms:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_DICOMParser:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_DomainsChemistry:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_DomainsChemistryOpenGL2:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_DomainsMicroscopy:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_DomainsParallelChemistry:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersAMR:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersCore:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_FiltersExtraction:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersFlowPaths:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersGeneral:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_FiltersGeneric:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersGeometry:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersHybrid:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersHyperTree:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersImaging:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersModeling:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersOpenTURNS:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersParallel:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersParallelDIY2:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersParallelFlowPaths:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersParallelGeometry:STRING=WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersParallelImaging:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersParallelMPI:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersParallelStatistics:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersParallelVerdict:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersPoints:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersProgrammable:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersReebGraph:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersSMP:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersSelection:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersSources:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_FiltersStatistics:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersTexture:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersTopology:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_FiltersVerdict:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_GUISupportQt:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_GUISupportQtQuick:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_GUISupportQtSQL:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_GeovisCore:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_GeovisGDAL:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOADIOS2:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOAMR:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOAsynchronous:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOCGNSReader:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOCONVERGECFD:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOChemistry:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOCityGML:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOCore:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_IOEnSight:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOExodus:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOExport:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_IOExportGL2PS:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOExportPDF:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOFFMPEG:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOFides:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOGDAL:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOGeoJSON:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOGeometry:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_IOH5Rage:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOH5part:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOHDF:STRING=WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOIOSS:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOImage:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_IOImport:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_IOInfovis:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOLAS:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOLSDyna:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOLegacy:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOMINC:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOMPIImage:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOMPIParallel:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOMotionFX:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOMovie:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOMySQL:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IONetCDF:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_IOODBC:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOOMF:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOOggTheora:STRING=WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOOpenVDB:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOPDAL:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOPIO:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOPLY:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOParallel:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOParallelExodus:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOParallelLSDyna:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOParallelNetCDF:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOParallelXML:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOParallelXdmf3:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOPostgreSQL:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOSQL:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOSegY:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOTRUCHAS:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOTecplotTable:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOVPIC:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOVeraOut:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOVideo:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOXML:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOXMLParser:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOXdmf2:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_IOXdmf3:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_ImagingColor:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_ImagingCore:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_ImagingFourier:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_ImagingGeneral:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_ImagingHybrid:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_ImagingMath:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_ImagingMorphological:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_ImagingOpenGL2:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_ImagingSources:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_ImagingStatistics:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_ImagingStencil:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_InfovisBoost:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_InfovisBoostGraphAlgorithms:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_InfovisCore:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_InfovisLayout:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_InteractionImage:STRING=WANT"
    "-DVTK_MODULE_ENABLE_VTK_InteractionStyle:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_InteractionWidgets:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_ParallelCore:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_ParallelDIY:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_ParallelMPI:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_Python:STRING=NO"
    "-DVTK_MODULE_ENABLE_VTK_PythonInterpreter:STRING=NO"
    "-DVTK_MODULE_ENABLE_VTK_RenderingAnnotation:STRING=DEFAULT"
    "-DVTK_MODULE_ENABLE_VTK_RenderingContext2D:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_RenderingContextOpenGL2:STRING=DEFAULT"
    "-DVTK_MODULE_ENABLE_VTK_RenderingCore:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_RenderingExternal:STRING=DEFAULT"
    "-DVTK_MODULE_ENABLE_VTK_RenderingFFMPEGOpenGL2:STRING=DEFAULT"
    "-DVTK_MODULE_ENABLE_VTK_RenderingFreeType:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_RenderingFreeTypeFontConfig:STRING=${_vtk_fontconfig}"
    "-DVTK_MODULE_ENABLE_VTK_RenderingGL2PSOpenGL2:STRING=DEFAULT"
    "-DVTK_MODULE_ENABLE_VTK_RenderingImage:STRING=WANT"
    "-DVTK_MODULE_ENABLE_VTK_RenderingLICOpenGL2:STRING=DEFAULT"
    "-DVTK_MODULE_ENABLE_VTK_RenderingLOD:STRING=DEFAULT"
    "-DVTK_MODULE_ENABLE_VTK_RenderingLabel:STRING=DEFAULT"
    "-DVTK_MODULE_ENABLE_VTK_RenderingMatplotlib:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_RenderingOpenGL2:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_RenderingOpenVR:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_RenderingParallel:STRING=WANT"
    "-DVTK_MODULE_ENABLE_VTK_RenderingParallelLIC:STRING=DEFAULT"
    "-DVTK_MODULE_ENABLE_VTK_RenderingQt:STRING=NO"
    "-DVTK_MODULE_ENABLE_VTK_RenderingRayTracing:STRING=DEFAULT"
    "-DVTK_MODULE_ENABLE_VTK_RenderingSceneGraph:STRING=DEFAULT"
    "-DVTK_MODULE_ENABLE_VTK_RenderingUI:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_RenderingVR:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_RenderingVolume:STRING=DEFAULT"
    "-DVTK_MODULE_ENABLE_VTK_RenderingVolumeAMR:STRING=DEFAULT"
    "-DVTK_MODULE_ENABLE_VTK_RenderingVolumeOpenGL2:STRING=DEFAULT"
    "-DVTK_MODULE_ENABLE_VTK_RenderingVtkJS:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_TestingCore:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_TestingGenericBridge:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_TestingIOSQL:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_TestingRendering:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_UtilitiesBenchmarks:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_ViewsContext2D:STRING=WANT"
    "-DVTK_MODULE_ENABLE_VTK_ViewsCore:STRING=WANT"
    "-DVTK_MODULE_ENABLE_VTK_ViewsInfovis:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_ViewsQt:STRING=NO"
    "-DVTK_MODULE_ENABLE_VTK_WebCore:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_WebGLExporter:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_WrappingPythonCore:STRING=NO"
    "-DVTK_MODULE_ENABLE_VTK_WrappingTools:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_cgns:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_cli11:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_diy2:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_doubleconversion:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_eigen:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_exodusII:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_expat:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_exprtk:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_fides:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_fmt:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_freetype:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_gl2ps:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_glew:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_h5part:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_hdf5:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_ioss:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_jpeg:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_jsoncpp:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_kissfft:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_kwiml:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_libharu:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_libproj:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_libxml2:STRING=WANT"

    # TODO(svenevs): can this actually be disabled?  If not do we need to patch
    # this or is it not actually something worth worrying about?
    # [18/2512] Building CXX object ThirdParty/loguru/vtkloguru/CMakeFiles/loguru.dir/loguru.cpp.o
    # /vtk/src/ThirdParty/loguru/vtkloguru/loguru.cpp: In function 'void vtkloguru::print_preamble(char*, size_t, vtkloguru::Ver
    # bosity, const char*, unsigned int)':
    # /vtk/src/ThirdParty/loguru/vtkloguru/loguru.cpp:1187:71: warning: '% 4d' directive output may be truncated writing between
    #  4 and 11 bytes into a region of size 5 [-Wformat-truncation=]
    #  1187 |                         snprintf(level_buff, sizeof(level_buff) - 1, "% 4d", verbosity);
    #       |                                                                       ^~~~
    # /vtk/src/ThirdParty/loguru/vtkloguru/loguru.cpp:1187:70: note: directive argument in the range [1, 2147483647]
    #  1187 |                         snprintf(level_buff, sizeof(level_buff) - 1, "% 4d", verbosity);
    #       |                                                                      ^~~~~~
    # In file included from /usr/include/stdio.h:894,

    "-DVTK_MODULE_ENABLE_VTK_loguru:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_lz4:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_lzma:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_metaio:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_netcdf:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_octree:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_ogg:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_opengl:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_pegtl:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_png:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_pugixml:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_sqlite:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_theora:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_tiff:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_utf8:STRING=WANT"
    "-DVTK_MODULE_ENABLE_VTK_verdict:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_vpic:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_vtkm:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_vtksys:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_xdmf2:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_xdmf3:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_zfp:STRING=DONT_WANT"
    "-DVTK_MODULE_ENABLE_VTK_zlib:STRING=YES"
    # Third party library control.
    "-DVTK_MODULE_USE_EXTERNAL_VTK_doubleconversion:BOOL=ON"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_eigen:BOOL=ON"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_expat:BOOL=${_vtk_use_external}"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_exprtk:BOOL=OFF"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_fmt:BOOL=OFF"

    # TODO(svenevs): something is going wrong with wheel / freetype.
    "-DVTK_MODULE_USE_EXTERNAL_VTK_freetype:BOOL=OFF"
    # "-DVTK_MODULE_USE_EXTERNAL_VTK_freetype:BOOL=${_vtk_use_external}"

    "-DVTK_MODULE_USE_EXTERNAL_VTK_glew:BOOL=${_vtk_use_external}"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_hdf5:BOOL=${_vtk_use_external}"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_jpeg:BOOL=ON"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_jsoncpp:BOOL=${_vtk_use_external}"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_libharu:BOOL=OFF"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_libxml2:BOOL=${_vtk_use_external}"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_lz4:BOOL=ON"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_lzma:BOOL=${_vtk_use_external}"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_netcdf:BOOL=${_vtk_use_external}"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_ogg:BOOL=${_vtk_use_external}"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_png:BOOL=ON"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_pugixml:BOOL=OFF"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_sqlite:BOOL=${_vtk_use_external}"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_theora:BOOL=${_vtk_use_external}"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_tiff:BOOL=ON"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_utf8:BOOL=OFF"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_zlib:BOOL=ON"
)
