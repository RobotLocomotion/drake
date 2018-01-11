{
  "Cps-Version": "0.8.0",
  "Name": "stx",
  "Description": "C++17 library facilities for older compilers",
  "License": "BSL-1.0",
  "Default-Components": [":stx"],
  "Components": {
    "stx": {
      "Type": "interface",
      "Includes": ["@prefix@/include/stx"]
    }
  }
}
