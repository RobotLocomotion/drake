import sys

from drake.tools.install.cpsutils import read_defs

# Targets correct as of the following list for upstream:
# https://github.com/pybind/pybind11/blob/9c0aa699/tools/pybind11Common.cmake#L8-L13

defs = read_defs(r"#define PYBIND11_(VERSION[^\s]+)\s+([^\s]+)")

if sys.platform.startswith('darwin'):
    defs["MODULE_LINK_FLAGS"] = '"Link-Flags": ["-undefined dynamic_lookup"],'
else:
    defs["MODULE_LINK_FLAGS"] = ""

# These are naively ported targets that are meant to ensure pybind11Tools does
# not fail fast. They may not match what is provided.
non_ported_targets = [
    "lto",
    "thin_lto",
    "python_link_helper",
    "python2_no_register",
    "windows_extras",
    "opt_size",
]
items = []
for target in non_ported_targets:
    items.append(f"""
    "{target}": {{
      "Type": "interface",
      "Includes": [
        "@prefix@/include/pybind11",
        "${{PYTHON_INCLUDE_DIRS}}"
      ],
      "Compile-Features": ["c++14"]
    }}""")
defs["NON_PORTED_TARGETS"] = ",\n".join(items)

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "pybind11",
  "Description": "Seamless operability between C++11 and Python",
  "License": "BSD-3-Clause",
  "Version": "%(VERSION_MAJOR)s.%(VERSION_MINOR)s.%(VERSION_PATCH)s",
  "Requires": {
    "PythonInterp": {
      "X-CMake-Find-Args": ["MODULE"]
    },
    "PythonLibs": {
      "X-CMake-Find-Args": ["MODULE"]
    }
  },
  "Default-Components": [":pybind11"],
  "Components": {
    "embed": {
      "Type": "interface",
      "Requires": [
        ":pybind11",
        "${PYTHON_LIBRARIES}"
      ]
    },
    "module": {
      "Type": "interface",
      %(MODULE_LINK_FLAGS)s
      "Requires": [":pybind11"]
    },
    "pybind11": {
      "Type": "interface",
      "Includes": [
        "@prefix@/include/pybind11",
        "${PYTHON_INCLUDE_DIRS}"
      ],
      "Compile-Features": ["c++14"]
    },
%(NON_PORTED_TARGETS)s
  },
  "X-CMake-Includes": ["${CMAKE_CURRENT_LIST_DIR}/pybind11Tools.cmake"],
  "X-CMake-Variables-Init": {
    "CMAKE_MODULE_PATH": "${CMAKE_CURRENT_LIST_DIR};${CMAKE_MODULE_PATH}"
  }
}
""" % defs

print(content[1:])
