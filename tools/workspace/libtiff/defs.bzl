load("//tools/skylark:cc.bzl", "cc_library")
load("@with_cfg.bzl", "with_cfg")

_builder = with_cfg(cc_library)
_builder.extend("copt", ["-w", "-fvisibility=hidden"])
_builder.extend("features", ["-supports_dynamic_linker"])
_builder.set(Label("@module_libtiff//:LIBDEFLATE_SUPPORT"), False)
libtiff_cc_library, _ = _builder.build()
