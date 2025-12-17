load(
    "//tools/workspace:cmake_configure_file.bzl",
    "cmake_configure_files",
)
load("//tools/workspace:generate_file.bzl", "generate_file")

# This list matches `vtk_numeric_types` in CMake/vtkTypeLists.cmake.
_VTK_NUMERIC_TYPES = (
    "char",
    "double",
    "float",
    "int",
    "long",
    "long long",
    "short",
    "signed char",
    "unsigned char",
    "unsigned int",
    "unsigned long",
    "unsigned long long",
    "unsigned short",
    "vtkIdType",
)

# This list matches `vtk_fixed_size_numeric_types` in CMake/vtkTypeLists.cmake.
_VTK_FIXED_SIZE_NUMERIC_TYPES = (
    "vtkTypeFloat32",
    "vtkTypeFloat64",
    "vtkTypeInt8",
    "vtkTypeInt16",
    "vtkTypeInt32",
    "vtkTypeInt64",
    "vtkTypeUInt8",
    "vtkTypeUInt16",
    "vtkTypeUInt32",
    "vtkTypeUInt64",
)

# The dict matches Common/Core/vtkTypeArrays.cmake's calls to `vtk_type_native`
# and `vtk_type_native_choice`. Each value is a tuple of (preferred_ctype,
# fallback_ctype).
_VTK_TYPE_NATIVE = {
    "vtkTypeInt8": ("signed char", None),
    "vtkTypeUInt8": ("unsigned char", None),
    "vtkTypeInt16": ("short", None),
    "vtkTypeUInt16": ("unsigned short", None),
    "vtkTypeInt32": ("int", None),
    "vtkTypeUInt32": ("unsigned int", None),
    "vtkTypeInt64": ("long", "long long"),
    "vtkTypeUInt64": ("unsigned long", "unsigned long long"),
    "vtkTypeFloat32": ("float", None),
    "vtkTypeFloat64": ("double", None),
}

def _generate_common_core_array_dispatch_array_list():
    """Mimics the vtkCreateArrayDispatchArrayList.cmake logic.
    Generates Common/Core/vtkArrayDispatchArrayList.h.
    """
    name = "common_core_array_dispatch_array_list"

    # This is hard-coded to use the default upstream options (e.g.,
    # VTK_DISPATCH_AOS_ARRAYS=ON, VTK_DISPATCH_SOA_ARRAYS=OFF, etc.).
    #
    # To update this content for newer versions of VTK, it's convenient to run
    # a CMake build and then inspect its generated vtkArrayDispatchArrayList.h.
    content = """
#pragma once
#include "vtkTypeList.h"
#include "vtkAOSDataArrayTemplate.h"
#include "vtkStructuredPointArray.h"
namespace vtkArrayDispatch {
VTK_ABI_NAMESPACE_BEGIN
using AOSArrays = vtkTypeList::Unique<vtkTypeList::Create<
""" + "\n".join([
        "vtkAOSDataArrayTemplate<{}>,".format(ctype)
        for ctype in _VTK_NUMERIC_TYPES
    ])[:-1] + """
>>::Result;
using SOAArrays = vtkTypeList::Unique<vtkTypeList::Create<>>::Result;
using ScaledSOAArrays = vtkTypeList::Unique<vtkTypeList::Create<>>::Result;
using ExtraArrays = vtkTypeList::Unique<vtkTypeList::Create<>>::Result;
using Arrays = vtkTypeList::Append<
  AOSArrays,
  SOAArrays,
  ScaledSOAArrays,
  ExtraArrays
>::Result;
using AffineArrays = vtkTypeList::Unique<vtkTypeList::Create<>>::Result;
using ConstantArrays = vtkTypeList::Unique<vtkTypeList::Create<>>::Result;
using StdFunctionArrays = vtkTypeList::Unique<vtkTypeList::Create<>>::Result;
using StridedArrays = vtkTypeList::Unique<vtkTypeList::Create<>>::Result;
using StructuredPointArrays = vtkTypeList::Unique<vtkTypeList::Create<
""" + "\n".join([
        "vtkStructuredPointArray<{}>,".format(ctype)
        for ctype in _VTK_NUMERIC_TYPES
    ])[:-1] + """
>>::Result;
using ImplicitExtraArrays = vtkTypeList::Unique<vtkTypeList::Create<>>::Result;
using ReadOnlyArrays = vtkTypeList::Append<
  AffineArrays,
  ConstantArrays,
  StdFunctionArrays,
  StridedArrays,
  StructuredPointArrays,
  ImplicitExtraArrays
>::Result;
using AllArrays = vtkTypeList::Append<Arrays, ReadOnlyArrays>::Result;
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

def _generate_common_core_type_list_macros():
    """Mimics the vtkCreateTypeListMacros.cmake logic.
    Generates Common/Core/vtkTypeListMacros.h.
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

def _generate_common_core_aos_typed_arrays():
    """Mimics a subset of the vtkTypeArrays.cmake logic, assuming a 64-bit
    platform. Generates an `*.h` and `*.cxx` file for each of VTK's primitive
    types.
    Returns the bulk_instantiation_srcs dictionary of generated files.
    """
    name = "common_core_aos_type_arrays"
    result_hdrs = []
    result_srcs = []
    bulk_instantiation_srcs = {}
    for vtk_type in _VTK_FIXED_SIZE_NUMERIC_TYPES:
        preferred_ctype, fallback_ctype = _VTK_TYPE_NATIVE[vtk_type]
        without_vtk_type_prefix = vtk_type.removeprefix("vtkType")
        preferred_ctype_upper = preferred_ctype.replace(" ", "_").upper()
        preferred_class = _ctype_to_vtk_camel_type(preferred_ctype)
        fallback_class = _ctype_to_vtk_camel_type(fallback_ctype)
        srcs = [
            "Common/Core/vtkAOSTypedArray.h.in",
            "Common/Core/vtkAOSTypedArray.cxx.in",
        ]
        outs = [
            "Common/Core/{}Array.h".format(vtk_type),
            # The CMakeLists.txt generates `*.cxx` files, but we don't want
            # Bazel to compile them so we use `*.inc` here.
            "Common/Core/{}Array.inc".format(vtk_type),
        ]
        cmake_configure_files(
            name = "_common_core_aos_type_arrays_" + without_vtk_type_prefix,
            srcs = srcs,
            outs = outs,
            defines = [
                "VTK_TYPE_NAME={}".format(without_vtk_type_prefix),
                "VTK_TYPE_NATIVE=" + """
#if VTK_TYPE_{vtk_type_upper} == VTK_{preferred_ctype_upper}
# include \"{preferred_class}Array.h\"
# define vtkTypeArrayBase {preferred_class}Array
#else
# include \"{fallback_class}Array.h\"
# define vtkTypeArrayBase {fallback_class}Array
#endif
                """.format(
                    vtk_type_upper = without_vtk_type_prefix.upper(),
                    preferred_ctype_upper = preferred_ctype_upper,
                    preferred_class = preferred_class,
                    fallback_class = fallback_class or "_ERROR_",
                ),
            ],
            strict = True,
        )
        result_hdrs.append(outs[0])
        result_srcs.append(outs[1])
        bulk_instantiation_srcs.setdefault(preferred_ctype, []).append(outs[1])
    native.filegroup(
        name = name + "_hdrs",
        srcs = result_hdrs,
    )
    native.filegroup(
        name = name + "_srcs",
        srcs = result_srcs,
    )
    return bulk_instantiation_srcs

def _generate_common_core_array_instantiations():
    """Mimics the Common/Core/CMakeLists.txt logic for {...}Instantiate.cxx.in
    codegen. Search for vtkAffineImplicitBackendInstantiate to find the relevant
    loop. Returns the bulk_instantiation_srcs dictionary of generated files.
    """
    name = "common_core_array_instantiations"
    all_outs = []
    bulk_instantiation_srcs = {}
    for ctype in _VTK_NUMERIC_TYPES:
        if ctype == "vtkIdType":
            continue
        suffix = ctype.replace(" ", "_")
        for prefix in (
            # This list matches Common/Core/CMakeLists.txt near the comment
            # "Order of this list is important with bulk instantiation".
            "vtkAffineImplicitBackendInstantiate",
            "vtkCompositeImplicitBackendInstantiate",
            "vtkConstantImplicitBackendInstantiate",
            "vtkIndexedImplicitBackendInstantiate",
            "vtkStridedImplicitBackendInstantiate",
            "vtkStructuredPointBackendInstantiate",
            "vtkAffineArrayInstantiate",
            "vtkAOSDataArrayTemplateInstantiate",
            "vtkCompositeArrayInstantiate",
            "vtkConstantArrayInstantiate",
            "vtkIndexedArrayInstantiate",
            "vtkScaledSOADataArrayTemplateInstantiate",
            "vtkStridedArrayInstantiate",
            "vtkSOADataArrayTemplateInstantiate",
            "vtkStdFunctionArrayInstantiate",
            "vtkStructuredPointArrayInstantiate",
            "vtkTypedDataArrayInstantiate",
            # This one is only instantiated iff "long" is part of the ctype.
            # This matches Common/Core/CMakeLists.txt near the comment "see
            # comments in vtkGenericDataArray.h for explanation".
            "vtkGenericDataArrayValueRangeInstantiate",
        ):
            if "Generic" in prefix and "long" not in ctype:
                # See comment immediately above.
                continue
            src = "Common/Core/{prefix}.cxx.in".format(prefix = prefix)

            # The CMakeLists.txt generates `*.cxx` files, but we don't want
            # Bazel to compile them so we use `*.inc` here.
            out = "Common/Core/{prefix}_{suffix}.inc".format(
                prefix = prefix,
                suffix = suffix,
            )
            cmake_configure_files(
                name = "_genrule_" + prefix + "_" + suffix,
                srcs = [src],
                outs = [out],
                defines = [
                    "INSTANTIATION_VALUE_TYPE=" + ctype,
                ],
                strict = True,
            )
            all_outs.append(out)
            bulk_instantiation_srcs.setdefault(ctype, []).append(out)
    native.filegroup(
        name = name,
        srcs = all_outs,
    )
    return bulk_instantiation_srcs

def _generate_common_core_typed_arrays():
    """Mimics a subset of the vtkTypeArrays.cmake logic, assuming a 64-bit
    platform. Generates an `*.h` and `*.cxx` file for each combination of VTK's
    primitive types and array backend types.
    Returns the bulk_instantiation_srcs dictionary of generated files.
    """
    name = "common_core_typed_arrays"
    result_hdrs = []
    result_srcs = []
    bulk_instantiation_srcs = {}
    for vtk_type in _VTK_FIXED_SIZE_NUMERIC_TYPES:
        ctype, _ = _VTK_TYPE_NATIVE[vtk_type]
        snake = ctype.replace(" ", "_")
        for backend in (
            "Affine",
            "Composite",
            "Constant",
            "Indexed",
            "ScaledSOA",
            "SOA",
            "StdFunction",
            "Strided",
        ):
            without_vtk_prefix = vtk_type[len("vtk"):]
            class_name = "vtk{}{}Array".format(backend, without_vtk_prefix)
            in_hdr = "Common/Core/vtk{}TypedArray.h.in".format(backend)
            out_hdr = "Common/Core/{}.h".format(class_name)
            in_src = "Common/Core/vtk{}TypedArray.cxx.in".format(backend)

            # The CMakeLists.txt generates `*.cxx` files, but we don't want
            # Bazel to compile them so we use `*.inc` here.
            out_src = "Common/Core/{}.inc".format(class_name)
            defines = [
                "CONCRETE_TYPE={}".format(vtk_type),
                "VTK_TYPE_NAME={}".format(without_vtk_prefix),
            ]
            if backend in (
                "Affine",
                "Composite",
                "Constant",
                "Indexed",
            ):
                # N.B. These types are deprecated in VTK 9.6 under the ctype
                # variants. This loop as currently written avoids the deprecated
                # configuration entirely (since it only generates sources for
                # vtk_types), and VTK itself has moved to their replacements.
                # However, VTK_DEPRECATION still appears in the .in files, so we
                # define it as empty.
                defines.append(
                    "VTK_DEPRECATION=",
                )
            cmake_configure_files(
                name = "_common_core_typed_arrays_{}_{}".format(
                    class_name,
                    vtk_type,
                ),
                srcs = [in_hdr, in_src],
                outs = [out_hdr, out_src],
                defines = defines,
                strict = True,
            )
            result_hdrs.append(out_hdr)
            result_srcs.append(out_src)
            bulk_instantiation_srcs.setdefault(ctype, []).append(out_src)
    native.filegroup(
        name = name + "_hdrs",
        srcs = result_hdrs,
    )
    native.filegroup(
        name = name + "_srcs",
        srcs = result_srcs,
    )
    return bulk_instantiation_srcs

def _merge_bulk_instantiation_srcs(*bulk_instantiation_srcs_args):
    """Given many bulk_instantiation_srcs dictionaries of generated files,
    returns a single merged dictionary.
    """
    result = {}
    for arg in bulk_instantiation_srcs_args:
        for ctype, files in arg.items():
            result.setdefault(ctype, []).extend(files)
    return result

def _generate_bulk_instantiation_srcs(bulk_instantiation_srcs):
    """Mimics Common/Core/CMakeLists.txt codegen of bulk instantiation source
    files Common/Core/vtkArrayBulkInstantiate_{suffix}.cxx. Search for
    BULK_INSTANTIATION_SOURCES to find the relevant loop.
    """
    all_outs = []
    for ctype in _VTK_NUMERIC_TYPES:
        src = "Common/Core/vtkArrayBulkInstantiate.cxx.in"
        out = "Common/Core/vtkArrayBulkInstantiate_{}.cxx".format(
            ctype.replace(" ", "_"),
        )
        cmake_configure_files(
            name = "_genrule_bulk_instantiation_srcs_" + ctype,
            srcs = [src],
            outs = [out],
            defines = [
                "BULK_INSTANTIATION_SOURCES=" + "\n".join([
                    "#include \"{}\"".format(x)
                    for x in bulk_instantiation_srcs.get(ctype, [])
                ]),
            ],
            strict = True,
        )
        all_outs.append(out)
    native.filegroup(
        name = "common_core_bulk_instantiation_srcs",
        srcs = all_outs,
    )

def generate_common_core_sources():
    _generate_common_core_array_dispatch_array_list()
    _generate_common_core_type_list_macros()
    bulk_instantiation_srcs1 = _generate_common_core_aos_typed_arrays()
    bulk_instantiation_srcs2 = _generate_common_core_array_instantiations()
    bulk_instantiation_srcs3 = _generate_common_core_typed_arrays()
    bulk_instantiation_srcs = _merge_bulk_instantiation_srcs(
        bulk_instantiation_srcs1,
        bulk_instantiation_srcs2,
        bulk_instantiation_srcs3,
    )
    _generate_bulk_instantiation_srcs(bulk_instantiation_srcs)
