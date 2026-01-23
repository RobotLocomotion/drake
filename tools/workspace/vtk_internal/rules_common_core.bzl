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
    "Int8": ("SIGNED_CHAR", None),
    "UInt8": ("UNSIGNED_CHAR", None),
    "Int16": ("SHORT", None),
    "UInt16": ("UNSIGNED_SHORT", None),
    "Int32": ("INT", None),
    "UInt32": ("UNSIGNED_INT", None),
    "Int64": ("LONG", "LONG_LONG"),
    "UInt64": ("UNSIGNED_LONG", "UNSIGNED_LONG_LONG"),
    "Float32": ("FLOAT", None),
    "Float64": ("DOUBLE", None),
}

def _generate_common_core_array_dispatch_array_list():
    """Mimics the vtkCreateArrayDispatchArrayList.cmake logic.
    Generates Common/Core/vtkArrayDispatchArrayList.h.
    """
    name = "common_core_array_dispatch_array_list"

    # This is hard-coded to use the default upstream options (e.g.,
    # VTK_DISPATCH_AOS_ARRAYS=ON, VTK_DISPATCH_SOA_ARRAYS=OFF, etc.).
    #
    # To update this content for newer versions of VTK, it's convenient to
    # run a CMake build (just the configure step) and then inspect its
    # generated vtkArrayDispatchArrayList.h.
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
        word.lower().capitalize()
        for word in ctype.split("_")
    ])

def _generate_common_core_type_list_macros():
    """Mimics the vtkCreateTypeListMacros.cmake logic.
    Generates Common/Core/vtkTypeListMacros.h.
    """
    name = "common_core_type_list_macros"
    max = 99  # Per Common/Core/CMakeLists.txt call to CreateTypeListMacros.
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
            # This one is instantiated iff "long" is part of the ctype.
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
            bulk_instantiation_srcs.setdefault(suffix.upper(), []).append(out)
    native.filegroup(
        name = name,
        srcs = all_outs,
    )
    return bulk_instantiation_srcs

def _generate_array_specialization(*, array_prefix, vtk_type, concrete_type):
    """Mimics a subset of vtkTypeArrays.cmake macro of the same name. Unlike
    CMakeLists.txt which generates a `*.cxx` file, we generate `*.inc` here
    because we don't want Bazel to compile it directly; instead, the `*.inc`
    file will be compiled via the "bulk instantiation" mechanism. Returns the
    generated code's filenames (out_hdr, out_src).
    """
    class_name = "vtk{}{}Array".format(array_prefix, vtk_type)
    in_hdr = "Common/Core/vtk{}TypedArray.h.in".format(array_prefix)
    out_hdr = "Common/Core/{}.h".format(class_name)
    in_src = "Common/Core/vtk{}TypedArray.cxx.in".format(array_prefix)
    out_src = "Common/Core/{}.inc".format(class_name)

    cmake_configure_files(
        name = "_common_core_typed_arrays_{}".format(class_name),
        srcs = [in_hdr, in_src],
        outs = [out_hdr, out_src],
        defines = [
            "CONCRETE_TYPE={}".format(concrete_type),
            "VTK_TYPE_NAME={}".format(vtk_type),
        ] + ([
            # VTK_DEPRECATION appears in a subset of the .in files, so we must
            # define it as empty. (Our build doesn't use the deprecated stuff.)
            "VTK_DEPRECATION=",
        ] if array_prefix in (
            "Affine",
            "Composite",
            "Constant",
            "Indexed",
        ) else []),
        strict = True,
    )
    return (out_hdr, out_src)

def _generate_common_core_typed_arrays():
    """Mimics a subset of vtkTypeArrays.cmake, for the (non-deprecated) loop
    that calls _generate_array_specialization. Generates a pair of `*.h` and
    `*.inc` files for the cartesian product of VTK's primitive types and array
    types. Returns the bulk_instantiation_srcs dictionary of generated files.
    """
    name = "common_core_typed_arrays"
    all_out_hdrs = []
    all_out_srcs = []
    bulk_instantiation_srcs = {}
    for array_prefix in (
        "Affine",
        "Composite",
        "Constant",
        "Indexed",
        "ScaledSOA",
        "SOA",
        "StdFunction",
        "Strided",
    ):
        for vtk_type in _VTK_FIXED_SIZE_NUMERIC_TYPES:
            without_vtk_prefix = vtk_type[len("vtk"):]
            out_hdr, out_src = _generate_array_specialization(
                array_prefix = array_prefix,
                vtk_type = without_vtk_prefix,
                concrete_type = vtk_type,
            )
            all_out_hdrs.append(out_hdr)
            all_out_srcs.append(out_src)
            ctype, _ = _VTK_TYPE_NATIVE[vtk_type[len("vtkType"):]]
            bulk_instantiation_srcs.setdefault(ctype, []).append(out_src)
    native.filegroup(
        name = name + "_hdrs",
        srcs = all_out_hdrs,
    )
    native.filegroup(
        name = name + "_srcs",
        srcs = all_out_srcs,
    )
    return bulk_instantiation_srcs

def _vtk_type_native(type):
    """Mimics the vtk_type_native and vtk_type_native_choice upstream functions.
    Note that instead of taking all of the same arguments as those functions,
    we take `type` and look up the rest.
    """
    preferred_ctype, fallback_ctype = _VTK_TYPE_NATIVE[type]
    type_upper = type.upper()
    if fallback_ctype == None:
        return """
        #if VTK_TYPE_{type_upper} == VTK_{ctype}
        # include \"{clazz}Array.h\"
        # define vtkTypeArrayBase {clazz}Array
        #endif
        """.format(
            type_upper = type_upper,
            ctype = preferred_ctype,
            clazz = _ctype_to_vtk_camel_type(preferred_ctype),
        )
    else:
        return """
#if VTK_TYPE_{type_upper} == VTK_{preferred_ctype}
# include \"{preferred_class}Array.h\"
# define vtkTypeArrayBase {preferred_class}Array
#elif VTK_TYPE_{type_upper} == VTK_{fallback_ctype}
# include \"{fallback_class}Array.h\"
# define vtkTypeArrayBase {fallback_class}Array
#endif
        """.format(
            type_upper = type_upper,
            preferred_ctype = preferred_ctype,
            preferred_class = _ctype_to_vtk_camel_type(preferred_ctype),
            fallback_ctype = fallback_ctype,
            fallback_class = _ctype_to_vtk_camel_type(fallback_ctype),
        )

def _generate_common_core_aos_typed_arrays():
    """Mimics a subset of vtkTypeArrays.cmake, for the loop that mentions
    vtkAOSTypedArray.h.in. Generates a pair of `*.h` and `*.inc` files for each
    of VTK's primitive types. Unlike CMakeLists.txt which generates a `*.cxx`
    file, we generate `*.inc` here because we don't want Bazel to compile it
    directly; instead, the `*.inc` file will be compiled via the "bulk
    instantiation" mechanism. Returns the bulk_instantiation_srcs dictionary of
    generated files.
    """
    name = "common_core_aos_type_arrays"
    all_out_hdrs = []
    all_out_srcs = []
    bulk_instantiation_srcs = {}
    for type in _VTK_FIXED_SIZE_NUMERIC_TYPES:
        vtk_type = type[len("vtkType"):]
        in_hdr = "Common/Core/vtkAOSTypedArray.h.in"
        out_hdr = "Common/Core/{}Array.h".format(type)
        in_src = "Common/Core/vtkAOSTypedArray.cxx.in"
        out_src = "Common/Core/{}Array.inc".format(type)
        cmake_configure_files(
            name = "_common_core_aos_type_arrays_" + type,
            srcs = [in_hdr, in_src],
            outs = [out_hdr, out_src],
            defines = [
                "VTK_TYPE_NAME={}".format(vtk_type),
                "VTK_TYPE_NATIVE=" + _vtk_type_native(vtk_type),
            ],
            strict = True,
        )
        all_out_hdrs.append(out_hdr)
        all_out_srcs.append(out_src)
        ctype, _ = _VTK_TYPE_NATIVE[vtk_type]
        bulk_instantiation_srcs.setdefault(ctype, []).append(out_src)
    native.filegroup(
        name = name + "_hdrs",
        srcs = all_out_hdrs,
    )
    native.filegroup(
        name = name + "_srcs",
        srcs = all_out_srcs,
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
    for ctype, files in bulk_instantiation_srcs.items():
        src = "Common/Core/vtkArrayBulkInstantiate.cxx.in"
        out = "Common/Core/vtkArrayBulkInstantiate_{}.cxx".format(ctype)
        cmake_configure_files(
            name = "_genrule_bulk_instantiation_srcs_" + ctype,
            srcs = [src],
            outs = [out],
            defines = [
                "BULK_INSTANTIATION_SOURCES=" + "\n".join([
                    "#include \"{}\"".format(x)
                    for x in files
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
