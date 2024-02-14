load("//tools/skylark:cc.bzl", "cc_library")
load("//tools/workspace:generate_file.bzl", "generate_file")
load(
    "//tools/workspace:cmake_configure_file.bzl",
    "cmake_configure_files",
)
load("@vtk_internal//:modules.bzl", "MODULES", "PLATFORM")
load("@vtk_internal//:settings.bzl", "MODULE_SETTINGS")

# You can manually set this to True, to get some feedback during upgrades.
_VERBOSE = False

def _bazelize_module_name(name):
    """Transforms e.g. `VTK::IOCore` => `VTK__IOCore` to make Bazel happy."""
    return name.replace(":", "_")

def _bazelize_module_names(names):
    """Maps _bazelize_module_name onto a list of names."""
    return [_bazelize_module_name(x) for x in names or []]

def _vtk_cc_module_impl(
        *,
        module_name,
        subdir,
        library_name = None,
        module_deps_public = [],
        module_deps_private = [],
        module_deps_ignore = [],
        hdrs_extra = [],
        hdrs_glob_exclude = [],
        hdrs_content = {},
        included_cxxs = [],
        srcs_extra = [],
        srcs_glob_extra = [],
        srcs_glob_exclude = [],
        srcs_objc_non_arc = [],
        cmake_defines_cmakelists = [],
        cmake_defines = [],
        cmake_undefines = [],
        defines_extra = [],
        strip_include_prefix_extra = "",
        copts_extra = [],
        linkopts_extra = [],
        deps_extra = [],
        visibility = []):
    """See vtk_cc_module() for documentation."""

    # Convert module names to bazel-safe names.
    module_name = _bazelize_module_name(module_name)
    module_deps_public = _bazelize_module_names(module_deps_public)
    module_deps_private = _bazelize_module_names(module_deps_private)

    # Cull any module deps that should be ignored.
    for dep_name in (module_deps_ignore or []):
        dep_name = _bazelize_module_name(dep_name)
        if dep_name in module_deps_public:
            module_deps_public.remove(dep_name)
        if dep_name in module_deps_private:
            module_deps_private.remove(dep_name)

    # Generate the module header, à la CMake GenerateExportHeader.
    if library_name != None and "ThirdParty" not in subdir:
        module_h = subdir + "/" + library_name + "Module.h"
        scream = library_name.upper()
        generate_file(
            name = module_h,
            content = "\n".join([
                "#pragma once",
                "#include \"vtkABINamespace.h\"",
                "#define {scream}_EXPORT",
                "#define {scream}_NO_EXPORT",
                "#define {scream}_DEPRECATED",
                "#define {scream}_DEPRECATED_EXPORT",
                "#define {scream}_DEPRECATED_NO_EXPORT",
            ]).format(scream = scream),
        )
        hdrs_extra = hdrs_extra + [module_h]

    for hdr_filename, hdr_content in (hdrs_content or {}).items():
        generate_file(name = hdr_filename, content = hdr_content)
        hdrs_extra = hdrs_extra + [hdr_filename]

    # Configure template headers (when necessary).
    gen_hdrs_lib = []
    subst_hdrs = native.glob([
        subdir + "/**/*.h.in",
        subdir + "/**/*.hpp.in",
        subdir + "/**/*.hxx.in",
        subdir + "/**/*.txx.in",
    ], exclude = [
        "**/test*",
    ] + hdrs_glob_exclude)
    gen_hdrs = [
        # We want to start from, e.g., "Common/Core/vtkDebug.h.in" and generate
        # "gen/Common/Core/vtkDebug.h" with CMake substitutions applied, which
        # means tacking a directory name on the front and stripping off ".in".
        "gen/" + x[:-3]
        for x in subst_hdrs
    ]
    if gen_hdrs:
        cmake_configure_files(
            name = "_genrule_hdrs_" + module_name,
            srcs = subst_hdrs,
            outs = gen_hdrs,
            cmakelists = cmake_defines_cmakelists,
            defines = cmake_defines,
            undefines = cmake_undefines,
            # TODO(jwnimmer-tri) Add opt-out config knob for strictness?
            strict = True,
        )
        cc_library(
            name = module_name + "_gen_hdrs",
            hdrs = gen_hdrs,
            strip_include_prefix = "gen/" + subdir,
            linkstatic = True,
        )
        gen_hdrs_lib = [module_name + "_gen_hdrs"]

    # Prepare the cc_library options.
    hdrs = hdrs_extra + included_cxxs + native.glob([
        subdir + "/**/*.h",
        subdir + "/**/*.hpp",
        subdir + "/**/*.hxx",
        subdir + "/**/*.txx",
    ], exclude = hdrs_glob_exclude, allow_empty = True)
    srcs = srcs_extra + native.glob(
        [subdir + "/*.cxx"] + srcs_glob_extra,
        exclude = included_cxxs + srcs_glob_exclude + [
            "**/vtkTest*",
            "**/test*",
        ],
        allow_empty = True,
    )
    copts = ["-w"] + copts_extra
    linkopts = linkopts_extra
    deps = []
    deps = deps + gen_hdrs_lib
    deps = deps + module_deps_public
    deps = deps + module_deps_private
    deps = deps + deps_extra

    # Deal with objc code.
    if PLATFORM["name"] != "linux" and srcs_objc_non_arc:
        objc_lib_name = "_" + module_name + "_objc"
        native.objc_library(
            name = objc_lib_name,
            non_arc_srcs = srcs_objc_non_arc,
            hdrs = hdrs,
            includes = [subdir],
            defines = defines_extra,
            copts = copts,
            linkopts = linkopts,
            deps = deps + [
                "//:_on_macos_you_must_not_have_forcepic_in_your_bazelrc_file_see_drake_issue_20217",  # noqa
            ],
        )
        deps = deps + [objc_lib_name]

    # For ASan & LSan, compile VTK without symbols (symbols are too big).
    copts = copts + select({
        "@drake//tools:using_sanitizer": ["-g0"],
        "//conditions:default": [],
    })
    features = select({
        "@drake//tools:using_sanitizer": ["-per_object_debug_info"],
        "//conditions:default": [],
    })

    # Declare the library using its upstream module name.
    cc_library(
        name = module_name,
        srcs = srcs,
        hdrs = hdrs,
        strip_include_prefix = subdir + strip_include_prefix_extra,
        defines = defines_extra,
        copts = copts,
        linkopts = linkopts,
        features = features,
        deps = deps,
        linkstatic = True,
    )

    # Alias the module name into the library name (if given).
    if library_name != None:
        native.alias(
            name = library_name,
            actual = module_name,
            visibility = visibility,
        )

def vtk_cc_module(
        *,
        module_metadata,
        module_settings = None):
    """
    Args:
        module_metadata: A parsed dict of the `vtk.module` contents, as
            returned by the parse_module() function in repository.bzl.
        module_settings: (Optional) A dict of customizations for how to
            build this module, typically loaded from a `settings.bzl` file.

    Keys of module_metadata:
        subdir: The directory path, e.g., `Foo/Bar`.
        NAME: The module name per VTK, e.g., `VTK::FooBar`.
        LIBRARY_NAME: (Optional) The library name per VTK, e.g., `vtkFooBar`.
        DEPENDS: (Optional) The public dependencies (by name).
        PRIVATE_DEPENDS: (Optional) The private dependencies (by name).

    Keys of module_settings (all of which are optional):
        module_deps_ignore: Dependencies (either public or private) to ignore.
            Typically this is used when the srcs have been trimmed to be
            narrower than the full set, i.e., when certain classes are being
            omitted from the build. The names here are VTK module names (e.g.,
            VTK::FooBar).
        hdrs_extra: Adds more headers beyond the default glob() patterns (*.h,
            *.hpp, *.hxx, *.txx, *.h.in, *.hpp.in, *.hxx.in, *.txx.in).
            Typically this is used for generated headers, since glob() only
            matches upstream files, not generated files.
        hdrs_glob_exclude: Skips these patterns. Typically this is used to skip
            configure headers that need special handling (e.g., `**/*.hxx.in`).
        hdrs_content: Map from source tree filename to generated file content.
            Useful to create configure headers directly instead of with CMake
            template substitution.
        included_cxxs: Flags cxxs filenames abused as headers; these files will
            be added to hdrs instead of srcs.
        srcs_extra: Adds more sources beyond the default glob() pattern
            (`{subdir}/*.cxx`). Typically this is either used to add files from
            from deeper subdirectories, or to opt back in to a narrow set of
            files that would otherwise been excluded by a srcs_glob_exclude.
        srcs_glob_extra: Adds to the default sources glob() pattern. Typically
            this is used to match files from from deeper subdirectories.
        srcs_glob_exclude: Skips these patterns. Typically this is used to skip
            classes that are unwanted or not necessary.
        srcs_objc_non_arc: Adds the given Objective-C sources to this library.
        cmake_defines: When generating a header file (e.g., foo.h.in => foo.h),
            uses these definitions. See cmake_configure_file() for details.
        cmake_undefines: When generating a header file, sets these definitions
            to be undefined. See cmake_configure_file() for details.
        defines_extra: Adds `defines = []` to the cc_library.
        strip_include_prefix_extra: Appends the given string after the default
            strip_include_prefix (i.e., the subdir name).
        copts_extra: Adds `copts = []` to the cc_library.
        linkopts_extra: Adds `linkopts = []` to the cc_library.
        deps_extra: Adds `deps = []` to the cc_library.
        visibility: Sets `visibility = []` on the cc_library.
    """

    # Delegate to impl, unpacking the module details in the function call.
    # This allows us to check the validity of all settings.bzl keywords.
    _vtk_cc_module_impl(
        module_name = module_metadata["NAME"],
        subdir = module_metadata["subdir"],
        library_name = module_metadata.get("LIBRARY_NAME"),
        module_deps_public = module_metadata.get("DEPENDS"),
        module_deps_private = module_metadata.get("PRIVATE_DEPENDS"),
        **(module_settings or {})
    )

def modules_closure(module_names, *, max_depth = 10):
    """Computes the transitive closure of the dependencies (both public and
    private) of the given model names. Both the argument and return value are
    a list of module_name strings.

    Skylark doesn't have `while` loops, so we need to specify a max search
    depth that's sufficient to cover the longest path in the deps graph.
    """
    result = []
    worklist = list(module_names)
    for _ in range(max_depth):
        if len(worklist) == 0:
            break
        next_worklist = []
        for name in worklist:
            result.append(name)
            settings = MODULE_SETTINGS.get(name, {})
            ignore = settings.get("module_deps_ignore", [])
            for kind in ["DEPENDS", "PRIVATE_DEPENDS"]:
                for dep_name in MODULES[name].get(kind, []):
                    if dep_name in result:
                        continue
                    if dep_name in worklist:
                        continue
                    if dep_name in next_worklist:
                        continue
                    if dep_name in ignore:
                        continue
                    next_worklist.append(dep_name)
        worklist = next_worklist
    if worklist:
        fail("Insufficient max_depth")
    return sorted(result)

def compile_all_modules():
    """Adds a cc_library rule for all modules in settings.bzl that are marked
    with non-default visibility, and also adds private cc_library rules for all
    of their required transitive dependency modules (per vtk.module metadata).
    """
    names = modules_closure([
        name
        for name, details in MODULE_SETTINGS.items()
        if "visibility" in details
    ])
    for name in names:
        vtk_cc_module(
            module_metadata = MODULES[name],
            module_settings = MODULE_SETTINGS.get(name),
        )

    # Optionally provide some diagnostics to help with VTK version upgrades.
    unused_settings = [
        name
        for name in MODULE_SETTINGS
        if name not in names
    ]
    modules_no_settings = [
        name
        for name in names
        if name not in MODULE_SETTINGS
    ]
    if _VERBOSE and unused_settings:
        print("settings.bzl has unused modules: " + str(unused_settings))
    if _VERBOSE and modules_no_settings:
        print("settings.bzl does not customize: " + str(modules_no_settings))

def generate_common_core_array_dispatch_array_list():
    """Mimics the vtkCreateArrayDispatchArrayList.cmake logic.
    Generates an `*.h` file.
    """
    name = "common_core_array_dispatch_array_list"

    # This is hard-coded to the default upstream options (AOS yes; SOA no).
    # We could parameterize this function with options if we ever need more.
    content = """
#pragma once
#include "vtkTypeList.h"
#include "vtkAOSDataArrayTemplate.h"
#include "vtkStructuredPointArray.h"
namespace vtkArrayDispatch {
VTK_ABI_NAMESPACE_BEGIN
typedef vtkTypeList::Unique<
  vtkTypeList::Create<
    vtkAOSDataArrayTemplate<char>,
    vtkAOSDataArrayTemplate<double>,
    vtkAOSDataArrayTemplate<float>,
    vtkAOSDataArrayTemplate<int>,
    vtkAOSDataArrayTemplate<long>,
    vtkAOSDataArrayTemplate<long long>,
    vtkAOSDataArrayTemplate<short>,
    vtkAOSDataArrayTemplate<signed char>,
    vtkAOSDataArrayTemplate<unsigned char>,
    vtkAOSDataArrayTemplate<unsigned int>,
    vtkAOSDataArrayTemplate<unsigned long>,
    vtkAOSDataArrayTemplate<unsigned long long>,
    vtkAOSDataArrayTemplate<unsigned short>,
    vtkAOSDataArrayTemplate<vtkIdType>
  >
>::Result Arrays;
typedef vtkTypeList::Unique<
  vtkTypeList::Create<
    vtkStructuredPointArray<double>
  >
>::Result ReadOnlyArrays;
typedef vtkTypeList::Unique<
  vtkTypeList::Append<
    Arrays,
    ReadOnlyArrays
  >::Result
>::Result AllArrays;
VTK_ABI_NAMESPACE_END
}
"""
    hdr = "Common/Core/vtkArrayDispatchArrayList.h"
    generate_file(name = hdr, content = content)
    native.filegroup(
        name = name,
        srcs = [hdr],
    )

def _ctype_to_vtk_camel_type(ctype):
    """Converts e.g. "unsigned short" to "vtkUnsignedShort".
    """
    if ctype == None:
        return None
    return "vtk" + "".join([
        word.capitalize()
        for word in ctype.split(" ")
    ])

def generate_common_core_type_list_macros():
    """Mimics the vtkCreateTypeListMacros.cmake logic.
    Generates an `*.h` file.
    """
    name = "common_core_type_list_macros"
    max = 99
    content = [
        "#pragma once",
        "#include \"vtkTypeList.h\"",
        "#define vtkTypeList_Create_1(t1) \\",
        "  vtkTypeList::TypeList<t1, vtkTypeList::NullType>",
    ]
    tail_list = []
    for i in range(2, max + 1):
        tail_list.append("t" + str(i))
        tail = ", ".join(tail_list)
        content.extend([
            "#define vtkTypeList_Create_" + str(i) + "(t1, " + tail + ") \\",
            "  vtkTypeList::TypeList<t1, \\",
            "    vtkTypeList_Create_" + str(i - 1) + "(" + tail + ") >",
        ])
    content.append("")
    hdr = "Common/Core/vtkTypeListMacros.h"
    generate_file(name = hdr, content = "\n".join(content))
    native.filegroup(
        name = name,
        srcs = [hdr],
    )

def generate_common_core_vtk_type_arrays():
    """Mimics the vtkTypeArrays.cmake logic, assuming a 64-bit platform.
    Generates an `*.h` and `*.cxx` file for each of VTK's primitive types.
    """
    name = "common_core_vtk_type_arrays"
    result_hdrs = []
    result_srcs = []
    for vtk_type, preferred_ctype, fallback_ctype in (
        ("Int8", "char", "signed char"),
        ("UInt8", "unsigned char", None),
        ("Int16", "short", None),
        ("UInt16", "unsigned short", None),
        ("Int32", "int", None),
        ("UInt32", "unsigned int", None),
        ("Int64", "long", "long long"),
        ("UInt64", "unsigned long", "unsigned long long"),
        ("Float32", "float", None),
        ("Float64", "double", None),
    ):
        preferred_ctype_upper = preferred_ctype.replace(" ", "_").upper()
        preferred_class = _ctype_to_vtk_camel_type(preferred_ctype)
        fallback_class = _ctype_to_vtk_camel_type(fallback_ctype)
        srcs = [
            "Common/Core/vtkTypedArray.h.in",
            "Common/Core/vtkTypedArray.cxx.in",
        ]
        outs = [
            "Common/Core/vtkType{}Array.h".format(vtk_type),
            "Common/Core/vtkType{}Array.cxx".format(vtk_type),
        ]
        cmake_configure_files(
            name = "_common_core_vtk_type_arrays_" + vtk_type,
            srcs = srcs,
            outs = outs,
            defines = [
                "VTK_TYPE_NAME={}".format(vtk_type),
                "VTK_TYPE_NATIVE=" + """
#if VTK_TYPE_{vtk_type_upper} == VTK_{preferred_ctype_upper}
# include \"{preferred_class}Array.h\"
# define vtkTypeArrayBase {preferred_class}Array
#else
# include \"{fallback_class}Array.h\"
# define vtkTypeArrayBase {fallback_class}Array
#endif
                """.format(
                    vtk_type_upper = vtk_type.upper(),
                    preferred_ctype_upper = preferred_ctype_upper,
                    preferred_class = preferred_class,
                    fallback_class = fallback_class or "_ERROR_",
                ),
            ],
            strict = True,
        )
        result_hdrs.append(outs[0])
        result_srcs.append(outs[1])
    native.filegroup(
        name = name + "_hdrs",
        srcs = result_hdrs,
    )
    native.filegroup(
        name = name + "_srcs",
        srcs = result_srcs,
    )

def generate_common_core_array_instantiations():
    """Mimic the instantiation_sources in Common/Core/CMakeLists.
    Generates a pile of headers.
    """
    name = "common_core_array_instantiations"
    result = []
    for ctype in (
        "char",
        "signed char",
        "unsigned char",
        "short",
        "unsigned short",
        "int",
        "unsigned int",
        "long",
        "unsigned long",
        "long long",
        "unsigned long long",
        "float",
        "double",
    ):
        for stem in (
            "vtkAffineArrayInstantiate",
            "vtkAffineImplicitBackendInstantiate",
            "vtkCompositeArrayInstantiate",
            "vtkCompositeImplicitBackendInstantiate",
            "vtkConstantArrayInstantiate",
            "vtkConstantImplicitBackendInstantiate",
            "vtkIndexedArrayInstantiate",
            "vtkIndexedImplicitBackendInstantiate",
            "vtkSOADataArrayTemplateInstantiate",
            "vtkStdFunctionArrayInstantiate",
            "vtkStructuredPointBackendInstantiate",
            "vtkStructuredPointArrayInstantiate",
            "vtkTypedDataArrayInstantiate",
            "vtkGenericDataArrayValueRangeInstantiate",
        ):
            if "Generic" in stem and "long" not in ctype:
                continue
            snake = ctype.replace(" ", "_")
            out = "Common/Core/{stem}_{snake}.cxx".format(
                stem = stem,
                snake = snake,
            )
            cmake_configure_files(
                name = "_genrule_" + stem + "_" + snake,
                srcs = ["Common/Core/" + stem + ".cxx.in"],
                outs = [out],
                defines = [
                    "INSTANTIATION_VALUE_TYPE=" + ctype,
                ],
                strict = True,
            )
            result.append(out)
    native.filegroup(
        name = name,
        srcs = result,
    )

def generate_common_core_sources():
    generate_common_core_array_dispatch_array_list()
    generate_common_core_type_list_macros()
    generate_common_core_vtk_type_arrays()
    generate_common_core_array_instantiations()

def cxx_embed(*, src, out, constant_name):
    """Mimics the vtkEncodeString.cmake logic.
    Generates an `*.h` file with the contents of a data file.
    """
    header = """
#pragma once
VTK_ABI_NAMESPACE_BEGIN
constexpr char {constant_name}[] = R"drakevtkinternal(
""".format(constant_name = constant_name)
    footer = """
)drakevtkinternal";
VTK_ABI_NAMESPACE_END
"""
    native.genrule(
        name = "_genrule_" + out,
        srcs = [src],
        outs = [out],
        cmd = " && ".join([
            "(echo '" + header + "' > $@)",
            "(cat $< >> $@)",
            "(echo '" + footer + "' >> $@)",
        ]),
    )

def _path_stem(src):
    """Returns e.g. "quux" when given "foo/bar/quux.ext".
    """
    return src.split("/")[-1].split(".")[0]

def generate_rendering_opengl2_sources():
    name = "generated_rendering_opengl2_sources"
    hdrs = []
    for src in native.glob([
        "Rendering/OpenGL2/glsl/*.glsl",
        "Rendering/OpenGL2/textures/*.jpg",
    ]):
        stem = _path_stem(src)
        hdr = "Rendering/OpenGL2/" + stem + ".h"
        cxx_embed(src = src, out = hdr, constant_name = stem)
        hdrs.append(hdr)
    native.filegroup(name = name, srcs = hdrs)
