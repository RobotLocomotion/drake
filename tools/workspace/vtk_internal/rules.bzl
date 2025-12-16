load("@vtk_internal//:modules.bzl", "MODULES", "PLATFORM")
load("@vtk_internal//:settings.bzl", "MODULE_SETTINGS")
load("//tools/skylark:cc.bzl", "cc_library", "objc_library")
load(
    "//tools/workspace:cmake_configure_file.bzl",
    "cmake_configure_files",
)
load("//tools/workspace:generate_file.bzl", "generate_file")

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
        includes_extra = [],
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
    ] + hdrs_glob_exclude, allow_empty = True)
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
            isystem = True,
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
            # Unwanted serialization code which leaks non-namespaced symbols.
            "**/*SerDesHelper.cxx",
            # Never build test code into our runtime libraries.
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
        objc_library(
            name = objc_lib_name,
            non_arc_srcs = srcs_objc_non_arc,
            hdrs = hdrs,
            defines = defines_extra,
            includes = [subdir] + includes_extra,
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
        defines = defines_extra,
        includes = includes_extra,
        isystem = True,
        strip_include_prefix = subdir + strip_include_prefix_extra,
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
        includes_extra: Adds `includes = []` to the cc_library.
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
                    if _VERBOSE:
                        print("Modules closure: {} uses {}".format(
                            name,
                            dep_name,
                        ))
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
