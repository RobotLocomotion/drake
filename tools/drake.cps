{
  "Name": "drake",
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
      ]
    }
  }
}
