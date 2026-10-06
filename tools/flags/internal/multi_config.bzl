load("@rules_python//python:py_binary.bzl", "py_binary")
load("@rules_python//python:py_test.bzl", "py_test")
load("@with_cfg.bzl", "with_cfg")

# The following stanzas define three new rules:
#
# py_test_with_alt_binder is equivalent to py_test except that it forces its
# dependencies to be built with --@drake//tools/flags:python_binder=nanobind.
# This allows us to run the test under the non-default binder setting as part
# of the same `bazel test` command as tests with the default binder setting.
#
# py_binary_with_alt_binder is the same idea, but for py_binary.
#
# python_binder_reset is like an alias() rule except that it clears the
# non-default flag setting. This is useful to mark dependencies that should
# not be rebuilt under the non-default binder setting (e.g., libdrake).

_ORIGINAL_SETTINGS = Label("//tools/flags/internal:alt_binder_original_settings")

_test_builder = with_cfg(py_test)
_test_builder.set(Label("//tools/flags:python_binder"), "nanobind")
_test_builder.resettable(_ORIGINAL_SETTINGS)
py_test_with_alt_binder, python_binder_reset = _test_builder.build()

_binary_builder = with_cfg(py_binary)
_binary_builder.set(Label("//tools/flags:python_binder"), "nanobind")
_binary_builder.resettable(_ORIGINAL_SETTINGS)
py_binary_with_alt_binder, _ = _binary_builder.build()
