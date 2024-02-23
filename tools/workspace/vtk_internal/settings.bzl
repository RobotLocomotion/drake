# This file contains configuration settings for how Drake should configure its
# private build of VTK.
#
# Each key in the MODULE_SETTINGS dict provides the settings for the VTK module
# of that name. If a module does not need any settings beyond the defaults, it
# can be omitted from the dict.
#
# The kinds of settings available for a module are documented in rules.bzl on
# the vtk_cc_module() rule.

MODULE_SETTINGS = {
    # First, we'll configure VTK's first-party utility libraries.
    "VTK::kwiml": {
        "cmake_defines": [
            # Match the Utilities/KWIML/vtkkwiml/CMakeLists.txt value. (This
            # version number hasn't changed in 8+ years, so we won't bother
            # automating it.)
            "KWIML_VERSION=1.0.0",
            "KWIML_VERSION_DECIMAL=1000000",
        ],
    },
    "VTK::vtksys": {
        # Upstream, this module has opt-in flags for each small feature within
        # the module. The VTK CMake script chooses which features to enable. To
        # match that logic here, we'll opt-out the default srcs glob and opt-in
        # to specific files that match how upstream VTK configures KWSys (with
        # exceptions noted inline below).
        "srcs_glob_exclude": ["**"],
        "srcs_extra": [
            # These files are enabled by default upstream, but Drake doesn't
            # need them:
            #  CommandLineArguments.cxx
            #  EncodingC.c
            #  EncodingCXX.cxx
            #  FStream.cxx
            #  Glob.cxx
            #  ProcessUNIX.c
            #  String.c
            #  System.c
            #  SystemInformation.cxx
            "Utilities/KWSys/vtksys/Base64.c",
            "Utilities/KWSys/vtksys/Directory.cxx",
            "Utilities/KWSys/vtksys/DynamicLoader.cxx",
            "Utilities/KWSys/vtksys/MD5.c",
            "Utilities/KWSys/vtksys/RegularExpression.cxx",
            "Utilities/KWSys/vtksys/Status.cxx",
            "Utilities/KWSys/vtksys/SystemTools.cxx",
        ],
        "cmake_defines": [
            # Match the VTK defaults.
            "KWSYS_NAMESPACE=vtksys",
            "KWSYS_NAME_IS_KWSYS=0",
            "KWSYS_SYSTEMTOOLS_USE_TRANSLATION_MAP=1",
            # Features that are available on the host platform.
            "KWSYS_STL_HAS_WSTRING=1",
            # Features that are NOT available on the host platform.
            "KWSYS_CXX_HAS_EXT_STDIO_FILEBUF_H=0",
            # The *module* prefix and suffix are the same on Linux and macOS.
            # https://gitlab.kitware.com/cmake/cmake/-/issues/21189
            "KWSYS_DynamicLoader_PREFIX=lib",
            "KWSYS_DynamicLoader_SUFFIX=.so",
            # Library API choices.
            "KWSYS_BUILD_SHARED=0",
        ],
        "copts_extra": [
            # Match the VTK defaults.
            "-DKWSYS_NAMESPACE=vtksys",
            # Features that are available on the host platform.
            "-DKWSYS_SYS_HAS_IFADDRS_H",
            "-DKWSYS_CXX_HAS_SETENV",
            "-DKWSYS_CXX_HAS_UNSETENV",
            "-DKWSYS_CXX_HAS_UTIMENSAT",
        ] + select({
            ":osx": [
                "-DKWSYS_CXX_STAT_HAS_ST_MTIMESPEC",
            ],
            "//conditions:default": [
                "-DKWSYS_CXX_STAT_HAS_ST_MTIM",
            ],
        }),
    },

    # Second, we'll configure the modules Drake needs (in alphabetical order).
    "VTK::CommonColor": {
        # This isn't used directly by Drake, but is used by other VTK modules.
    },
    "VTK::CommonComputationalGeometry": {
        # This module has a lot of code we don't need. We'll opt-out of the
        # default srcs glob, and instead just specify what Drake needs.
        "srcs_glob_exclude": ["**"],
        "srcs_extra": [
            "Common/ComputationalGeometry/vtkKochanekSpline.cxx",
        ],
    },
    "VTK::CommonCore": {
        "visibility": ["//visibility:public"],
        "hdrs_glob_exclude": [
            # These header files are consumed by bespoke configure_file logic,
            # so we don't want to automatically configure them. Instead, we'll
            # use generate_common_core_sources() from rules.bzl to handle it.
            "Common/Core/vtkArrayDispatchArrayList.h.in",
            "Common/Core/vtkTypeListMacros.h.in",
            "Common/Core/vtkTypedArray.h.in",
        ],
        "hdrs_extra": [
            # These are the hdrs outputs of generate_common_core_sources() from
            # rules.bzl (related to the hdrs_glob_exclude, immediately above).
            ":common_core_array_dispatch_array_list",
            ":common_core_type_list_macros",
            ":common_core_vtk_type_arrays_hdrs",
        ],
        "included_cxxs": [
            "Common/Core/vtkMersenneTwister_Private.cxx",
        ],
        "srcs_extra": [
            # Sources in nested subdirs are not globbed by default, so we need
            # to list the nested sources we want explicitly.
            "Common/Core/SMP/Common/vtkSMPToolsAPI.cxx",
            "Common/Core/SMP/Sequential/vtkSMPToolsImpl.cxx",
            # These header files are generated by custom configure_file logic.
            # See generate_common_core_sources() in rules.bzl.
            ":common_core_array_instantiations",
            ":common_core_vtk_type_arrays_srcs",
        ],
        "srcs_glob_exclude": [
            # Optional files that we choose not to enable.
            "Common/Core/vtkAndroid*",
            "Common/Core/vtkWin32*",
        ],
        "cmake_defines_cmakelists": [
            # Scrape the VTK_..._VERSION definitions from this file.
            "CMake/vtkVersion.cmake",
        ],
        "cmake_defines": [
            # Emulate the concatenation found in the root CMakeLists.txt.
            "VTK_VERSION=@VTK_MAJOR_VERSION@.@VTK_MINOR_VERSION@.@VTK_BUILD_VERSION@",  # noqa
            # ABI
            "VTK_HAS_ABI_NAMESPACE=1",
            "VTK_ABI_NAMESPACE_NAME=drake_vendor",
            "VTK_ABI_NAMESPACE_BEGIN=inline namespace drake_vendor __attribute__ ((visibility (\"hidden\"))) {",  # noqa
            "VTK_ABI_NAMESPACE_END=}",
            # Features that are available on the host platform.
            "VTK_HAS_CXXABI_DEMANGLE=1",
            "VTK_HAS_ISFINITE=1",
            "VTK_HAS_ISINF=1",
            "VTK_HAS_ISNAN=1",
            "VTK_HAS_STD_ISFINITE=1",
            "VTK_HAS_STD_ISINF=1",
            "VTK_HAS_STD_ISNAN=1",
            "VTK_USE_64BIT_IDS=1",
            "VTK_USE_64BIT_TIMESTAMPS=1",
            # Threading.
            "VTK_MAX_THREADS=1",
            "VTK_SMP_DEFAULT_IMPLEMENTATION_SEQUENTIAL=1",
            "VTK_SMP_ENABLE_SEQUENTIAL=1",
            "VTK_SMP_IMPLEMENTATION_TYPE=Sequential",
            "VTK_USE_PTHREADS=1",
            # Library API choices.
            "VTK_ALL_NEW_OBJECT_FACTORY=1",
            "VTK_ALWAYS_OPTIMIZE_ARRAY_ITERATORS=1",
            "VTK_LEGACY_REMOVE=1",
            "VTK_USE_FUTURE_BOOL=1",
            "VTK_USE_FUTURE_CONST=1",
            "VTK_WARN_ON_DISPATCH_FAILURE=1",
        ],
        "cmake_undefines": [
            # Features that are NOT available on the host platform.
            "VTK_HAS_FEENABLEEXCEPT",
            "VTK_HAS_FINITE",
            "VTK_HAS__FINITE",
            "VTK_HAS__ISNAN",
            "VTK_REQUIRE_LARGE_FILE_SUPPORT",
            # All of Drake's supported CPUs are little-endian. If we ever do
            # need to support, this we can patch the header file to always
            # use the built-in __BIG_ENDIAN__ (instead of only on macOS).
            "VTK_WORDS_BIGENDIAN",
            # Threading.
            "VTK_SMP_DEFAULT_IMPLEMENTATION_OPENMP",
            "VTK_SMP_DEFAULT_IMPLEMENTATION_STDTHREAD",
            "VTK_SMP_DEFAULT_IMPLEMENTATION_TBB",
            "VTK_SMP_ENABLE_OPENMP",
            "VTK_SMP_ENABLE_STDTHREAD",
            "VTK_SMP_ENABLE_TBB",
            "VTK_USE_WIN32_THREADS",
            # Library API choices.
            "VTK_BUILD_SHARED_LIBS",
            "VTK_DEBUG_LEAKS",
            "VTK_DEBUG_RANGE_ITERATORS",
            "VTK_DISPATCH_AFFINE_ARRAYS",
            "VTK_DISPATCH_CONSTANT_ARRAYS",
            "VTK_DISPATCH_STD_FUNCTION_ARRAYS",
            "VTK_DISPATCH_STRUCTURED_POINT_ARRAYS",
            "VTK_LEGACY_SILENT",
            "VTK_USE_MEMKIND",
            "VTK_USE_SCALED_SOA_ARRAYS",
        ],
    },
    "VTK::CommonDataModel": {
        "visibility": ["//visibility:public"],
        "included_cxxs": [
            "Common/DataModel/vtkHyperTreeGridNonOrientedMooreSuperCursorData.inl",  # noqa
            "Common/DataModel/vtkHyperTreeGridNonOrientedVonNeumannSuperCursorData.inl",  # noqa
        ],
    },
    "VTK::CommonExecutionModel": {
        "visibility": ["//visibility:public"],
    },
    "VTK::CommonMath": {
        "visibility": ["//visibility:public"],
        # This module has a lot of code we don't need. We'll opt-out of the
        # default srcs glob, and instead just specify what Drake needs.
        "srcs_glob_exclude": ["**"],
        "srcs_extra": [
            "Common/Math/vtkFunctionSet.cxx",
            "Common/Math/vtkMatrix3x3.cxx",
            "Common/Math/vtkMatrix4x4.cxx",
            "Common/Math/vtkQuaternionInterpolator.cxx",
        ],
        "module_deps_ignore": [
            "VTK::kissfft",
        ],
    },
    "VTK::CommonMisc": {
        # This module has a lot of code we don't need. We'll opt-out of the
        # default srcs glob, and instead just specify what Drake needs.
        "srcs_glob_exclude": ["**"],
        "srcs_extra": [
            "Common/Misc/vtkContourValues.cxx",
            "Common/Misc/vtkErrorCode.cxx",
            "Common/Misc/vtkHeap.cxx",
        ],
        "module_deps_ignore": [
            "VTK::exprtk",
        ],
    },
    "VTK::CommonSystem": {
        # This module has a lot of code we don't need. We'll opt-out of the
        # default srcs glob, and instead just specify what Drake needs.
        "srcs_glob_exclude": ["**"],
        "srcs_extra": [
            "Common/System/vtkDirectory.cxx",
            "Common/System/vtkTimerLog.cxx",
        ],
    },
    "VTK::CommonTransforms": {
        "visibility": ["//visibility:public"],
    },
    "VTK::FiltersCore": {
        "visibility": ["//visibility:public"],
        # This module has a lot of code we don't need. We'll opt-out of the
        # default srcs glob, and instead just specify what Drake needs.
        "srcs_glob_exclude": ["**"],
        "srcs_extra": [
            "Filters/Core/vtkAppendPolyData.cxx",
            "Filters/Core/vtkDecimatePro.cxx",
            "Filters/Core/vtkGlyph3D.cxx",
            "Filters/Core/vtkPolyDataNormals.cxx",
            "Filters/Core/vtkPolyDataTangents.cxx",
            "Filters/Core/vtkTriangleFilter.cxx",
        ],
    },
    "VTK::FiltersGeneral": {
        "visibility": ["//visibility:public"],
        # This module has a lot of code we don't need. We'll opt-out of the
        # default srcs glob, and instead just specify what Drake needs.
        "srcs_glob_exclude": ["**"],
        "srcs_extra": [
            "Filters/General/vtkExtractSelectedFrustum.cxx",
            "Filters/General/vtkExtractSelectionBase.cxx",
            "Filters/General/vtkGraphToPoints.cxx",
            "Filters/General/vtkIconGlyphFilter.cxx",
            "Filters/General/vtkImageDataToPointSet.cxx",
            "Filters/General/vtkRectilinearGridToPointSet.cxx",
            "Filters/General/vtkSphericalHarmonics.cxx",
            "Filters/General/vtkTransformFilter.cxx",
            "Filters/General/vtkTransformPolyDataFilter.cxx",
            "Filters/General/vtkVertexGlyphFilter.cxx",
        ],
        "module_deps_ignore": [
            "VTK::FiltersVerdict",
        ],
    },
    "VTK::FiltersGeometry": {
        # This module has a lot of code we don't need. We'll opt-out of the
        # default srcs glob, and instead just specify what Drake needs.
        "srcs_glob_exclude": ["**"],
        "srcs_extra": [
            "Filters/Geometry/vtkDataSetSurfaceFilter.cxx",
            "Filters/Geometry/vtkGeometryFilter.cxx",
            "Filters/Geometry/vtkRectilinearGridGeometryFilter.cxx",
            "Filters/Geometry/vtkStructuredGridGeometryFilter.cxx",
            "Filters/Geometry/vtkUnstructuredGridGeometryFilter.cxx",
        ],
    },
    "VTK::FiltersHybrid": {
        # This module has a lot of code we don't need. We'll opt-out of the
        # default srcs glob, and instead just specify what Drake needs.
        "srcs_glob_exclude": ["**"],
        "srcs_extra": [
            "Filters/Hybrid/vtkWeightedTransformFilter.cxx",
        ],
        "included_cxxs": [
            "Filters/Hybrid/vtkEarthSource.cxx",
            "Filters/Hybrid/vtkEarthSourceData.inl",
        ],
        "module_deps_ignore": [
            "VTK::ImagingSources",
        ],
    },
    "VTK::FiltersSources": {
        "visibility": ["//visibility:public"],
        "srcs_glob_exclude": [
            # Avoid the use of VTK::CommonComputationalGeometry.
            "**/vtkPartitionedDataSetCollectionSource.cxx",
            "**/vtkPartitionedDataSetSource.cxx",
            # Avoid some VTK::FiltersGeneral stuff we don't need.
            "**/vtkSpatioTemporalHarmonicsSource.cxx",
        ],
    },
    "VTK::IOCore": {
        "srcs_glob_exclude": [
            # Skip code we don't need.
            "**/*Glob*",
            "**/*Particle*",
            "**/*Java*",
            "**/*UTF*",
            # Skip this to avoid a dependency on lz4.
            "**/*LZ4*",
            # Skip this to avoid a dependency on lzma.
            "**/*LZMA*",
            # Skip this to avoid a dependency on utf8.
            "**/*Codec*",
        ],
        "module_deps_ignore": [
            "VTK::lz4",
            "VTK::lzma",
            "VTK::utf8",
        ],
    },
    "VTK::IOExport": {
        "visibility": ["//visibility:public"],
        # This module has a lot of code we don't need. We'll opt-out of the
        # default srcs glob, and instead just specify what Drake needs.
        "srcs_glob_exclude": ["**"],
        "srcs_extra": [
            "IO/Export/vtkExporter.cxx",
            "IO/Export/vtkGLTFExporter.cxx",
        ],
        "module_deps_ignore": [
            "VTK::DomainsChemistry",
            "VTK::FiltersCore",
            "VTK::FiltersGeometry",
            "VTK::IOImage",
            "VTK::IOXML",
            "VTK::ImagingCore",
            "VTK::RenderingContext2D",
            "VTK::RenderingFreeType",
            "VTK::RenderingVtkJS",
            "VTK::libharu",
            "VTK::utf8",
        ],
    },
    "VTK::IOGeometry": {
        "visibility": ["//visibility:public"],
        # This module has a lot of code we don't need. We'll opt-out of the
        # default srcs glob, and instead just specify what Drake needs.
        "srcs_glob_exclude": ["**"],
        "srcs_extra": [
            "IO/Geometry/vtkGLTFDocumentLoader.cxx",
            "IO/Geometry/vtkGLTFDocumentLoaderInternals.cxx",
            "IO/Geometry/vtkGLTFReader.cxx",
            "IO/Geometry/vtkGLTFUtils.cxx",
            "IO/Geometry/vtkGLTFWriter.cxx",
            "IO/Geometry/vtkGLTFWriterUtils.cxx",
            "IO/Geometry/vtkOBJWriter.cxx",
            "IO/Geometry/vtkSTLReader.cxx",
        ],
        "module_deps_ignore": [
            "VTK::FiltersVerdict",
        ],
    },
    "VTK::IOImage": {
        "visibility": ["//visibility:public"],
        # This module has a lot of code we don't need. We'll opt-out of the
        # default srcs glob, and instead just specify what Drake needs.
        "srcs_glob_exclude": ["**"],
        "srcs_extra": [
            "IO/Image/vtkHDRReader.cxx",
            "IO/Image/vtkImageExport.cxx",
            "IO/Image/vtkImageReader.cxx",
            "IO/Image/vtkImageReader2.cxx",
            "IO/Image/vtkImageReader2Collection.cxx",
            "IO/Image/vtkImageReader2Factory.cxx",
            "IO/Image/vtkImageWriter.cxx",
            "IO/Image/vtkJPEGReader.cxx",
            "IO/Image/vtkJPEGWriter.cxx",
            "IO/Image/vtkPNGReader.cxx",
            "IO/Image/vtkPNGWriter.cxx",
            "IO/Image/vtkTIFFReader.cxx",
            "IO/Image/vtkTIFFWriter.cxx",
        ],
        "module_deps_ignore": [
            "VTK::DICOMParser",
            "VTK::metaio",
        ],
    },
    "VTK::IOImport": {
        "visibility": ["//visibility:public"],
        # This module has a lot of code we don't need. We'll opt-out of the
        # default srcs glob, and instead just specify what Drake needs.
        "srcs_glob_exclude": ["**"],
        "srcs_extra": [
            "IO/Import/vtkImporter.cxx",
            "IO/Import/vtkGLTFImporter.cxx",
        ],
    },
    "VTK::IOLegacy": {
        "visibility": ["//visibility:public"],
        # This module has a lot of code we don't need. We'll opt-out of the
        # default srcs glob, and instead just specify what Drake needs.
        "srcs_glob_exclude": ["**"],
        "srcs_extra": [
            "IO/Legacy/vtkDataReader.cxx",
            "IO/Legacy/vtkUnstructuredGridReader.cxx",
        ],
    },
    "VTK::ImagingCore": {
        "visibility": ["//visibility:public"],
    },
    "VTK::RenderingCore": {
        "visibility": ["//visibility:public"],
        "copts_extra": [
            # Match the VTK defaults.
            "-DVTK_OPENGL2",
        ],
    },
    "VTK::RenderingOpenGL2": {
        "visibility": ["//visibility:public"],
        "cmake_defines": select({
            ":osx": [
                "VTK_USE_COCOA",
            ],
            "//conditions:default": [
                "VTK_USE_X",
            ],
        }),
        "cmake_undefines": [
            "VTK_DEFAULT_RENDER_WINDOW_OFFSCREEN",
            "VTK_OPENGL_ENABLE_STREAM_ANNOTATIONS",
            "VTK_OPENGL_HAS_EGL",
            "VTK_OPENGL_HAS_OSMESA",
            "VTK_REPORT_OPENGL_ERRORS",
            "VTK_REPORT_OPENGL_ERRORS_IN_RELEASE_BUILDS",
            "VTK_USE_CORE_GRAPHICS",
            "VTK_USE_DIRECTX",
            "VTK_USE_NVCONTROL",
        ] + select({
            ":osx": [
                "VTK_USE_X",
            ],
            "//conditions:default": [
                "VTK_USE_COCOA",
            ],
        }),
        "hdrs_extra": [
            ":generated_rendering_opengl2_sources",
        ],
        "srcs_glob_exclude": [
            # This is configure-time setup code, not library code.
            "**/vtkProbe*",
            # Avoid building unnecessary VTK::RenderingHyperTreeGrid.
            "**/*HyperTreeGrid*",
            # Exclude all renderers by default; we'll incorporate the necessary
            # ones using with srcs_extra immediately below.
            "**/vtkCocoa*",
            "**/vtkEGL*",
            "**/vtkOSOpenGL*",
            "**/vtkSDL2OpenGL*",
            "**/vtkWin32OpenGL*",
            "**/vtkXOpenGL*",
        ],
        "srcs_objc_non_arc": select({
            ":osx": [
                "Rendering/OpenGL2/vtkCocoaGLView.mm",
                "Rendering/OpenGL2/vtkCocoaRenderWindow.mm",
            ],
            "//conditions:default": [],
        }),
        "srcs_extra": select({
            ":osx": [],
            "//conditions:default": [
                "Rendering/OpenGL2/vtkXOpenGLRenderWindow.cxx",
            ],
        }) + [
            # The vtkObjectFactory.cmake logic for vtk_object_factory_configure
            # is too difficult to implement in Bazel at the moment. Instead,
            # we'll commit the two generated files and directly mention them.
            "@drake//tools/workspace/vtk_internal:gen/vtkRenderingOpenGL2ObjectFactory.h",  # noqa
            "@drake//tools/workspace/vtk_internal:gen/vtkRenderingOpenGL2ObjectFactory.cxx",  # noqa
        ],
        "linkopts_extra": select({
            ":osx": [
                # Mimic vtk_module_link(... "-framework Cocoa") from upstream.
                "-framework Cocoa",
            ],
            "//conditions:default": [],
        }),
        "deps_extra": select({
            ":osx": [],
            "//conditions:default": [
                # Mimic vtk_module_link(... X11::X11) from upstream.
                "@x11",
            ],
        }),
        "module_deps_ignore": [
            "VTK::RenderingHyperTreeGrid",
        ],
    },
    "VTK::RenderingUI": {
        # This module has a lot of code we don't need. We'll opt-out of the
        # default srcs glob, and instead just specify what Drake needs.
        "srcs_glob_exclude": ["**"],
        "srcs_extra": [
            "Rendering/UI/vtkGenericRenderWindowInteractor.cxx",
        ],
    },

    # Third, we'll configure dependencies that come from Drake's WORKSPACE.
    "VTK::fmt": {
        "cmake_defines": [
            "VTK_MODULE_USE_EXTERNAL_VTK_fmt=1",
        ],
        "deps_extra": [
            "@fmt",
        ],
    },
    "VTK::jpeg": {
        "cmake_defines": [
            "VTK_MODULE_USE_EXTERNAL_vtkjpeg=1",
        ],
        "hdrs_glob_exclude": [
            "ThirdParty/jpeg/vtkjpeg/**",
        ],
        "deps_extra": [
            "@libjpeg_turbo_internal//:jpeg",
        ],
    },
    "VTK::nlohmannjson": {
        "cmake_defines": [
            "VTK_MODULE_USE_EXTERNAL_vtknlohmannjson=1",
        ],
        "deps_extra": [
            "@nlohmann_internal//:nlohmann",
        ],
    },
    "VTK::opengl": {
        "deps_extra": [
            "@opengl",
        ],
    },
    "VTK::png": {
        "cmake_defines": [
            "VTK_MODULE_USE_EXTERNAL_vtkpng=1",
        ],
        "deps_extra": [
            "@libpng_internal//:libpng",
        ],
    },
    "VTK::tiff": {
        "cmake_defines": [
            "VTK_MODULE_USE_EXTERNAL_vtktiff=1",
        ],
        "deps_extra": [
            "@libtiff_internal//:libtiff",
        ],
    },
    "VTK::zlib": {
        "cmake_defines": [
            "VTK_MODULE_USE_EXTERNAL_vtkzlib=1",
        ],
        "deps_extra": [
            "@zlib",
        ],
    },

    # Fourth, we'll configure dependencies that we let VTK build and vendor on
    # its own, because nothing else in Drake needs these. Anything added here
    # must have its license file added to the install in `package.BUILD.bazel`.
    # VTK's name mangling of these is a little bit weak (it just adds "vtk" to
    # the front, leaving them as public symbols); we might want to improve upon
    # that later on.
    "VTK::doubleconversion": {
        "cmake_undefines": [
            "VTK_MODULE_USE_EXTERNAL_vtkdoubleconversion",
        ],
        "hdrs_content": {
            "ThirdParty/doubleconversion/vtkdoubleconversion_export.h": """
                #pragma once
                #define VTKDOUBLECONVERSION_EXPORT
            """,
        },
        "srcs_glob_extra": [
            "ThirdParty/doubleconversion/**/*.cc",
        ],
    },
    "VTK::fast_float": {
        "cmake_undefines": [
            "VTK_MODULE_USE_EXTERNAL_vtkfast_float",
        ],
    },
    "VTK::glew": {
        "cmake_undefines": [
            "VTK_GLEW_SHARED",
            "VTK_MODULE_USE_EXTERNAL_vtkglew",
            "VTK_MODULE_vtkglew_GLES3",
        ],
        "srcs_extra": [
            "ThirdParty/glew/vtkglew/src/glew.c",
        ],
        "copts_extra": [
            "-Iexternal/vtk_internal/ThirdParty/glew/vtkglew/include",
            # Match the target_compile_definitions() from CMakeLists.txt.
            "-DGLEW_NO_GLU",
        ],
        "linkopts_extra": select({
            ":osx": [],
            "//conditions:default": [
                "-lX11",
                "-lGLX",
            ],
        }),
    },
    "VTK::pugixml": {
        # TODO(jwnimmer-tri) The only user of pugixml is vtkDataAssembly.
        # Possibly there is some way to disable XML I/O support on that
        # class, so that we can drop this dependency.
        "cmake_undefines": [
            "VTK_MODULE_USE_EXTERNAL_vtkpugixml",
        ],
        "hdrs_glob_exclude": [
            # We use `hdrs_content` instead of a full-blown configure file.
            "**/pugiconfig.hpp.in",
        ],
        "hdrs_content": {
            "ThirdParty/pugixml/pugiconfig.hpp": """
                #pragma once
                #define PUGIXML_API
            """,
        },
        "srcs_glob_extra": [
            "ThirdParty/pugixml/**/*.cpp",
        ],
    },
    "VTK::token": {
        "hdrs_glob_exclude": [
            # We use `hdrs_content` instead of a full-blown configure file.
            "**/Options.h.in",
            "**/CxxABIConfigure.h.in",
        ],
        "hdrs_content": {
            "ThirdParty/token/vtktoken/token/CxxABIConfigure.h": """
#pragma once
#define token_HAS_CXXABI_DEMANGLE
#include <cxxabi.h>
""",
            "ThirdParty/token/vtktoken/token/Exports.h": """
#pragma once
#define TOKEN_EXPORT
#define TOKEN_NO_EXPORT
#define TOKEN_DEPRECATED
#define TOKEN_DEPRECATED_EXPORT
#define TOKEN_DEPRECATED_NO_EXPORT
            """,
            "ThirdParty/token/vtktoken/token/Options.h": """
#pragma once
#define token_NAMESPACE vtktoken
#define token_BEGIN_NAMESPACE namespace vtktoken \
  __attribute__ ((visibility (\"hidden\"))) {
#define token_CLOSE_NAMESPACE }
            """,
        },
        "strip_include_prefix_extra": "/vtktoken",
        "srcs_glob_extra": [
            "ThirdParty/token/vtktoken/token/*.cxx",
        ],
        "srcs_glob_exclude": [
            # Don't link the main() program.
            "**/tokenize.cxx",
        ],
    },
}
