load(":common.bzl", "globvalue", "error", "success", "write_build")
load(":wrapped_ctx.bzl", "repo_path", "repo_symlink")

def _find_home(repo_ctx):
  varname = repo_ctx.attr.envvar
  if varname in repo_ctx.os.environ:
    return success(repo_path(repo_ctx, repo_ctx.os.environ[varname]))
  if repo_ctx.attr.default:
    return success(repo_path(repo_ctx, repo_ctx.attr.default))
  tmpl = "Environment variable '{}' not set and no default specified"
  return error(tmpl.format(varname))

def setup_local_cc_library(repo_ctx):
  home = _find_home(repo_ctx)
  if home.error != None:
    return home
  paths = [home.value.get_child(p) for p in ["lib", "include"]]
  for child in paths:
    if not child.exists:
      return error("{} not found".format(child))
  for child in paths:
    repo_symlink(repo_ctx, child, child.basename)
  # The values below should result in the following behavior:
  # 1) "fully static" builds will be that, for any combination of .a and .so
  # 2) "mostly static" builds will link statically against the lib if a .a is
  #    present, dynamically otherwise.
  # 3) "dynamic" builds will link dynamically if a .so is present.
  #
  # All dynamic linking should prefer the specified path, but will
  # use the system search path as well.
  write_build(repo_ctx,
              # In theory, Bazel supports versioned .so files,
              # but in practice including them breaks static linking.
              srcs=globvalue(["lib/*.a", "lib/*.so", "lib/*.dylib"]),
              defines=repo_ctx.attr.defines,
              includes=["include"],
              linkopts=["-Wl,-rpath " + str(paths[0])])
  return success(True)

def _impl(repo_ctx):
  error = setup_local_cc_library(repo_ctx).error
  if error != None:
    fail(error)

local_cc_library_configure = repository_rule(
    _impl,
    attrs = {
        "envvar": attr.string(mandatory = True),
        "default": attr.string(),
        "defines": attr.string_list(),
        "build_file_template": attr.label(
            default = Label("@kythe//tools/build_rules/config:BUILD.tpl"),
            single_file = True,
            allow_files = True,
        ),
    },
    local = True,
)
