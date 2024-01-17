# These are the Apple-specific CMake definitions we want to enable for Drake.
# Keep this list alpha-sorted.
_APPLE_CMAKE_DEFINES = [
    "HAVE_ARC4RANDOM_BUF=1",
]

# These are the Linux-specific CMake definitions we want to enable for Drake.
# Keep this list alpha-sorted.
_LINUX_CMAKE_DEFINES = [
    "HAVE_GETRANDOM=1",
]

# These are the settings we want to enable for Drake. A few of them towards the
# bottom are conditional for Apple vs Linux. Keep this list alpha-sorted.
CMAKE_DEFINES = [
    "PACKAGE_NAME=\"expat\"",
    "XML_CONTEXT_BYTES=1024",
    "XML_GE=1",
] + select({
    "@drake//tools/cc_toolchain:apple": _APPLE_CMAKE_DEFINES,
    "@drake//tools/cc_toolchain:linux": _LINUX_CMAKE_DEFINES,
    "//conditions:default": [],
})

# These are the settings we want to disable for Drake on all platforms.
# Keep this list alpha-sorted.
CMAKE_UNDEFINES = [
]
