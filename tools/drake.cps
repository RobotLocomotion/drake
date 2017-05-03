{
  "Meta-Comment": "Common Package Specification for Drake",
  "Meta-Schema": "https://mwoehlke.github.io/cps/",
  "X-Purpose": "Used to generate drake-config.cmake via cps2cmake",
  "X-See-Also": "https://github.com/mwoehlke/pycps",
  "Cps-Version": "0.8.0",
  "Name": "drake",
  "Website": "http://drake.mit.edu/",
  "Components": {
    "drake": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libdrake.so",
      "Includes": [
        "@prefix@/include",
        "@prefix@/include/external/bullet",
        "@prefix@/include/external/eigen",
        "@prefix@/include/external/fmt",
        "@prefix@/include/external/spdlog",
        "@prefix@/include/external/tinyobjloader"
      ],
      "Compile-Features": ["c++14"]
    }
  }
}
