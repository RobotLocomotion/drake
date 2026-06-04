load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@with_cfg.bzl", "with_cfg")

_builder = with_cfg(cc_library)
_builder.extend("copt", ["-fvisibility=hidden"])
_builder.extend("features", ["-supports_dynamic_linker"])
cc_static_hidden_library, _ = _builder.build()
