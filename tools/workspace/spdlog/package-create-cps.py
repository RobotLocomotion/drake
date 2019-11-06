from drake.tools.install.cpsutils import read_defs, read_requires

def_re = "#define\s+(\S+)\s+(\S+)"
defs = read_defs(def_re)

defs.update(read_requires())

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "spdlog",
  "Description": "Fast C++ logging library",
  "License": "MIT",
  "Version": "%(SPDLOG_VER_MAJOR)s.%(SPDLOG_VER_MINOR)s.%(SPDLOG_VER_PATCH)s",
  "Requires": {
    "fmt": {
      "Version": "%(fmt_VERSION)s",
      "Hints": ["@prefix@/lib/cmake/fmt"],
      "X-CMake-Find-Args": ["CONFIG"]
    }
  },
  "Default-Components": [":spdlog"],
  "Components": {
    "spdlog": {
      "Type": "dylib",
      "Includes": ["@prefix@/include/spdlog"],
      "Location": "@prefix@/lib/libspdlog.so",
      "Definitions": [
        "HAVE_SPDLOG",
        "SPDLOG_COMPILED_LIB",
        "SPDLOG_FMT_EXTERNAL"
      ],
      "Requires": ["fmt:fmt-header-only"]
    }
  }
}
""" % defs

print(content[1:])
