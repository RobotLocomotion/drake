{
  "Cps-Version": "0.8.0",
  "Name": "protobuf",
  "Description": "Protocol Buffers - Google's data interchange format",
  "License": "BSD-2-Clause",
  "Version": "3.1.0",
  "Default-Components": [":protobuf", ":protobuf_lite", ":protoc_lib"],
  "Components": {
    "protobuf": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libprotobuf.so",
      "Includes": ["@prefix@/include"]
    },
    "protobuf_lite": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libprotobuf_lite.so",
      "Includes": ["@prefix@/include"]
    },
    "protoc_lib": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libprotoc_lib.so",
      "Includes": ["@prefix@/include"],
      "Requires": [":libprotobuf"]
    },
    "protoc": {
      "Type": "exe",
      "Location": "@prefix@/bin/protoc"
    }
  }
}
