load("@rules_python//python:py_test.bzl", "py_test")
load("@with_cfg.bzl", "with_cfg")

# The following stanza defines two new rules:
#
# py_test_with_alt_binder is equivalent to py_test except that it forces its
# dependencies to be built with --@drake//tools/flags:python_binder=nanobind.
# This allows us to run the test under the non-default binder setting as part
# of the same `bazel test` command as tests with the default binder setting.
#
# python_binder_reset is like an alias() rule except that it clears the
# non-default flag setting. This is useful to mark dependencies that should
# not be rebuilt under the non-default binder setting (e.g., libdrake).

_builder = with_cfg(py_test)
_builder.set(Label("//tools/flags:python_binder"), "nanobind")
_builder.resettable(Label("//tools/flags/internal:py_test_with_alt_binder_original_settings"))
py_test_with_alt_binder, python_binder_reset = _builder.build()
