import sys

from drake.tools.install.cpsutils import read_defs

# Targets correct as of pybind11/pybind11@6d0b470. Ensure that SHA is taken
# from pybind11/pybind11 and not RobotLocomotion/pybind11.

defs = read_defs("#define PYBIND11_(VERSION[^\s]+)\s+([^\s]+)")

if sys.platform.startswith('darwin'):
    defs["MODULE_LINK_FLAGS"] = '"Link-Flags": ["-undefined dynamic_lookup"],'
else:
    defs["MODULE_LINK_FLAGS"] = ""

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
    }
  },
  "X-CMake-Includes": ["${CMAKE_CURRENT_LIST_DIR}/pybind11Tools.cmake"],
  "X-CMake-Variables-Init": {
    "CMAKE_MODULE_PATH": "${CMAKE_CURRENT_LIST_DIR};${CMAKE_MODULE_PATH}"
  }
}
""" % defs

print(content[1:])
