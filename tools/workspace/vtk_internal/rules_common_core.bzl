load(
    "//tools/workspace:cmake_configure_file.bzl",
    "cmake_configure_files",
)
load("//tools/workspace:generate_file.bzl", "generate_file")

def _generate_common_core_array_dispatch_array_list():
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

def _generate_common_core_type_list_macros():
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

def _generate_common_core_vtk_type_arrays():
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

def _generate_common_core_array_instantiations():
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
        snake = ctype.replace(" ", "_")
        bulk_srcs = []
        for stem in (
            "vtkAffineImplicitBackendInstantiate",
            "vtkCompositeImplicitBackendInstantiate",
            "vtkConstantImplicitBackendInstantiate",
            "vtkIndexedImplicitBackendInstantiate",
            "vtkStructuredPointBackendInstantiate",
            "vtkAffineArrayInstantiate",
            "vtkCompositeArrayInstantiate",
            "vtkConstantArrayInstantiate",
            "vtkIndexedArrayInstantiate",
            "vtkSOADataArrayTemplateInstantiate",
            "vtkStdFunctionArrayInstantiate",
            "vtkStructuredPointArrayInstantiate",
            "vtkTypedDataArrayInstantiate",
            "vtkTypeArrayInstantiate",
            # This one is only instantiated iff "long" is part of the ctype.
            "vtkGenericDataArrayValueRangeInstantiate",
        ):
            if "Generic" in stem and "long" not in ctype:
                continue

            # The CMakeLists.txt generates `*.cxx` files, but we don't want
            # Bazel to compile them so we use `*.inc` here.
            out = "Common/Core/{stem}_{snake}.inc".format(
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
            bulk_srcs.append(out)
            result.append(out)
        out = "Common/Core/vtkArrayBulkInstantiate_" + snake + ".cxx"
        cmake_configure_files(
            name = "_genrule_bulk_srcs_" + snake,
            srcs = ["Common/Core/vtkArrayBulkInstantiate.cxx.in"],
            outs = [out],
            defines = [
                "BULK_INSTANTIATION_SOURCES=" + "\n".join([
                    "#include \"{}\"".format(x)
                    for x in bulk_srcs
                ]),
            ],
            strict = True,
        )
        result.append(out)
    native.filegroup(
        name = name,
        srcs = result,
    )

def _generate_common_core_implicit_arrays():
    """Mimics a subset of the vtkImplicitArrays.cmake logic.
    Generates `*.h` and `*.cxx` files.
    """
    name = "common_core_implicit_arrays"
    result_hdrs = []
    result_srcs = []
    for backend in ["Constant"]:
        for ctype in (
            "unsigned char",
        ):
            cased_type = _ctype_to_vtk_camel_type(ctype)[len("vtk"):]
            class_name = "vtk{}{}Array".format(backend, cased_type)
            in_hdr = "Common/Core/vtk{}TypedArray.h.in".format(backend)
            out_hdr = "Common/Core/{}.h".format(class_name)
            in_src = "Common/Core/vtk{}TypedArray.cxx.in".format(backend)
            out_src = "Common/Core/{}.cxx".format(class_name)
            cmake_configure_files(
                name = "_genrule_implicit_arrays_{}".format(class_name),
                srcs = [in_hdr, in_src],
                outs = [out_hdr, out_src],
                defines = [
                    "CONCRETE_TYPE={}".format(ctype),
                    "VTK_TYPE_NAME={}".format(cased_type),
                ],
                strict = True,
            )
            result_hdrs.append(out_hdr)
            result_srcs.append(out_src)
    native.filegroup(
        name = name + "_hdrs",
        srcs = result_hdrs,
    )
    native.filegroup(
        name = name + "_srcs",
        srcs = result_srcs,
    )

def generate_common_core_sources():
    _generate_common_core_array_dispatch_array_list()
    _generate_common_core_type_list_macros()
    _generate_common_core_vtk_type_arrays()
    _generate_common_core_array_instantiations()
    _generate_common_core_implicit_arrays()
