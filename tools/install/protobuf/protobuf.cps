{
  "Cps-Version": "0.8.0",
  "Name": "protobuf",
  "Description": "Language-neutral, platform-neutral, extensible mechanism for serializing structured data",
  "License": "BSD-3-Clause",
  "Version": "3.1.0",
  "Default-Components": [":protobuf"],
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
