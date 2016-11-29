"""C System Library dependencies.

This package exports a repository rule and macro.

cc_system_package_configure:
  This repository_rule combines behavior of both
  local_cc_library_configure and pkg_config_package rules.
  It first attempts to find the specified environment variable.
  If present, it tries to use a local library directory at that path.
  If absent or the rule fails, it attempts to use `pkg-config` to configure
  the dependency.
  Finally, if a default path is specified it will attempt to use that
  as a local directory before failing.

cc_system_package:
  This is a macro which munges the name passed into a 'local_{name}'
  repository, whose `:lib` target is then bound to '{name}'.
"""

load("@//tools/third_party/kythe/tools/build_rules/config:wrapped_ctx.bzl", "wrapctx")
load("@//tools/third_party/kythe/tools/build_rules/config:local.bzl", "setup_local_cc_library")
load("@//tools/third_party/kythe/tools/build_rules/config:pkg_config.bzl", "setup_pkg_config_package")

def try_local_library(repo_ctx):
  if repo_ctx.attr.envvar and repo_ctx.attr.envvar in repo_ctx.os.environ:
    error = setup_local_cc_library(repo_ctx).error
    if not error:
      return None
    print(repo_ctx.attr.envvar, "defined but unusable:", error)
  return ""  # Printed error as warning.

def try_pkg_config(repo_ctx):
  return setup_pkg_config_package(repo_ctx).error

def try_default(repo_ctx):
  if not repo_ctx.attr.default:
    return ""
  return setup_local_cc_library(repo_ctx).error

def _impl(repo_ctx):
  errors = []
  for setup in [try_local_library, try_pkg_config, try_default]:
    error = setup(repo_ctx)
    if error == None:
      return
    elif error:  # Ignore "empty" errors.
      errors += [error]
  fail("\n".join(errors))

cc_system_package_configure = repository_rule(
    _impl,
    attrs = {
        "modname": attr.string(mandatory = True),
        "atleast_version": attr.string(),
        "max_version": attr.string(),
        "exact_version": attr.string(),
        "envvar": attr.string(),
        "default": attr.string(),
        "defines": attr.string_list(),
        "build_file_template": attr.label(
            default = Label("@//tools/third_party/kythe/tools/build_rules/config:BUILD.tpl"),
            single_file = True,
            allow_files = True,
        ),
    },
    local = True,
)

def cc_system_package(*, name, **kwargs):
  if 'modname' not in kwargs:
    kwargs['modname'] = name
  cc_system_package_configure(name="local_" + name, **kwargs)
  native.bind(name=name, actual="@local_{name}//:lib".format(name=name))
