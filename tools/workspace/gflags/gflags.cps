{
  "Cps-Version": "0.8.0",
  "Name": "gflags",
  "Description": "A C++ library that implements command line flag processing",
  "License": "BSD-3-Clause",
  "Version": "2.2.0",
  "Default-Components": [":gflags"],
  "Components": {
    "gflags": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libgflags.so",
      "Includes": ["@prefix@/include"]
    }
  }
}
