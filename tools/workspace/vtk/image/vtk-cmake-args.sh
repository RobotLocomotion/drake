# shellcheck shell=bash
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
# shellcheck disable=SC2154
if ! [[ "${drake_vtk_build_flavor}" =~ ^(linux|linux_wheel|mac|mac_wheel)$ ]]; then
    echo "vtk-cmake-args.sh: the 'drake_vtk_build_flavor' variable must be " >&2
    echo "set to one of 'linux', 'linux_wheel', 'mac', or 'mac_wheel'.  " >&2
    echo "Example: drake_vtk_build_flavor='linux' . ./vtk-cmake-args.sh" >&2
    exit 1
fi
# shellcheck disable=SC2154
if [[ -z "${codename}" ]]; then
    echo "vtk-cmake-args.sh: the 'codename' variable was expected to be " >&2
    echo "set, but had no value.  Source common-definitions.sh." >&2
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
    "-DVTK_MODULE_ENABLE_VTK_IONetCDF:STRING=NO"
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
    "-DVTK_MODULE_ENABLE_VTK_IOParallelNetCDF:STRING=NO"
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
    "-DVTK_MODULE_ENABLE_VTK_RenderingFreeTypeFontConfig:STRING=DONT_WANT"
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
    # Third party library module control.
    "-DVTK_MODULE_ENABLE_VTK_kwiml:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_opengl:STRING=YES"
    "-DVTK_MODULE_ENABLE_VTK_octree:STRING=YES"  # Needed for rendering.
    "-DVTK_MODULE_ENABLE_VTK_metaio:STRING=YES"  # Needed for IOImage.
    "-DVTK_MODULE_ENABLE_VTK_vtkm:STRING=NO"
    "-DVTK_MODULE_ENABLE_VTK_vtksys:STRING=YES"
)

# Third party dependencies.

# Dependencies other parts of drake depend on directly get shared.  For the
# .tar.gz builds these come from apt-get

# TODO(svenevs): add a comment somewhere to coordinate the deps between the
# files so that developers keep everything in sync.  Maybe to README?
# - macOS
#     - setup/mac/binary_distribution/Brewfile
#     - tools/wheel/image/packages-macos
# - ubuntu
#     - tools/workspace/vtk/image/prereqs
#     - setup/ubuntu/binary_distribution/packages-focal.txt
#     - setup/ubuntu/binary_distribution/packages-jammy.txt
#     - tools/wheel/image/packages-focal
readonly shared_external_dependencies=(
    eigen
    png
    zlib
)
for external in "${shared_external_dependencies[@]}"; do
    vtk_cmake_args+=(
        "-DVTK_MODULE_ENABLE_VTK_${external}:STRING=YES"
        # ON: effectively requests VTK `find_package(${external} REQUIRED)`.
        "-DVTK_MODULE_USE_EXTERNAL_VTK_${external}:BOOL=ON"
    )
done
# On macOS find_package(...) will potentially find the *wrong* third party
# dependency (e.g., there is a zlib vendored with Xcode).  Force the correct
# location for these third party libraries.
if [[ "${drake_vtk_build_flavor}" =~ ^(mac|mac_wheel)$ ]]; then
    vtk_cmake_args+=(
        "-DEigen3_ROOT=\"$(brew --prefix eigen)\""
        "-DPNG_ROOT=\"$(brew --prefix png)\""
        "-DZLIB_ROOT=\"$(brew --prefix zlib)\""
    )
fi

# Third party libraries that are only needed for VTK are built by VTK.  They are
# linked statically, VTK mangles the namespaces / C symbols.  This is *NOT*
# included in the inline namespace drake uses to hide the rest of the VTK
# symbols.
readonly private_external_dependencies=(
    doubleconversion
    expat
    exprtk
    fmt
    glew
    hdf5
    jpeg
    jsoncpp
    kissfft
    libharu
    libxml2
    loguru
    lz4
    lzma
    sqlite
    utf8
    freetype
    tiff
)
for external in "${private_external_dependencies[@]}"; do
    vtk_cmake_args+=(
        "-DVTK_MODULE_ENABLE_VTK_${external}:STRING=YES"
        # OFF: VTK will vendor via `add_subdirectory(ThirdParty/${external})`.
        "-DVTK_MODULE_USE_EXTERNAL_VTK_${external}:BOOL=OFF"
    )
done

# TODO(svenevs): re-gather the list of vtk third party modules we want to
# prevent from being added forcfully.  Metaio / netcdf? pugixml theora
readonly prevent_external_dependencies=(
    cgns
    cli11
    diy2
    exodusII
    fides
    gl2ps
    h5part
    ioss
    libproj
    netcdf
    ogg
    pegtl
    verdict
    vpic
    xdmf3
    zfp
)
for external in "${prevent_external_dependencies[@]}"; do
    vtk_cmake_args+=(
        "-DVTK_MODULE_ENABLE_VTK_${external}:STRING=NO"
        # ON: effectively requests VTK `find_package(${external} REQUIRED)`.
        "-DVTK_MODULE_USE_EXTERNAL_VTK_${external}:BOOL=ON"
    )
done

# TODO(svenevs): add loop that checks the arrays and makes sure there is no
# overlap.  If developer adds something to both shared_external_libraries and
# private_external_libraries, the private version will triumph (cmake args come
# afterward).  Same for `prevent`...
