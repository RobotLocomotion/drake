# -*- python -*-

# TODO(6996) Once the dust settles down, remove this convenience forwarding,
# for consistency with the other drake_{py,java,...} languages.
load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_test",
)
