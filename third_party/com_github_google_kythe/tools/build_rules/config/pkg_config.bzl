"""Skylark module for system libraries known to pkg-config.

pkg_config_module(name, modname,
                  atleast_version, exact_version, max_version)
  Create a local repository based on the results of pkg-config.

  Args:
    name: string. The name of the repository.
    modname: string. The name of the pkg-config module.
    atleast_version: Optional, string. Require at least this version.
    exact_version: Optional, string. Require exactly this version.
    max_version: Optional, string.  Require less than this version.

  The configured repository will have a `cc_library` target with the
  provided `name`.
"""

load(":common.bzl", "error", "success", "write_build")
load(":wrapped_ctx.bzl", "unwrap")

_cps_build_content = """
load(
    "@drake//tools/install:install.bzl",
    "cmake_config",
    "install",
    "install_cmake_config",
)

CMAKE_PACKAGE = "{package}"

cmake_config(
    cps_file_name = CMAKE_PACKAGE + ".cps",
    package = CMAKE_PACKAGE,
)

install_cmake_config(package = CMAKE_PACKAGE)

install(
    name = "install",
    workspace = CMAKE_PACKAGE,
    visibility = ["//visibility:public"],
    deps = [":install_cmake_config"],
)
"""

_cps_content = """
{{
  "Cps-Version": "0.8.0",
  "Name": "{package}",
  "Description": "Library for collision detection between two convex shapes",
  "License": "{license}",
  "Version": "{vers}",
  "Default-Components": [":{package}"],
  "Components": {{
    "{package}": {{
      "Type": "dylib",
      "Location": "{libdir}/lib{package}.{extension}",
      "Includes": ["{include}"]
    }}
  }}
}}
"""

def _write_build(repo_ctx, cflags, linkopts, install_content):
  # Silence a warning about unknown cflags.
  if "-pthread" in cflags and "-pthread" in linkopts:
    cflags.remove("-pthread")
  includes, defines = _parse_cflags(repo_ctx, cflags)
  write_build(repo_ctx, includes, defines, linkopts, install_content)

def _fail(repo_ctx, message, tail=""):
  """Fail with message if repo_ctx.attr.mandatory, otherwise warn."""
  modname = repo_ctx.attr.modname
  if tail:
    message = "\nError processing {}: {}\n\n{}".format(modname, message, tail)
  else:
    message = "\nError processing {}: {}".format(modname, message)
  return error(message)

def _find(repo_ctx):
  pkg_config = unwrap(repo_ctx).which("pkg-config")
  if pkg_config == None:
    return _fail(repo_ctx, "Unable to find pkg-config executable")
  return success(pkg_config)

def _exists(repo_ctx, pc_args):
  result = unwrap(repo_ctx).execute(pc_args + ["--print-errors", "--exists"])
  if result.return_code != 0:
    return _fail(repo_ctx, "Unable to find module", result.stderr)
  return success(True)

def _installed_version(repo_ctx, pc_args):
  result = unwrap(repo_ctx).execute(pc_args + ["--modversion"])
  if result.return_code != 0:
    return _fail(repo_ctx, "Unable to determine installed version", result.stderr)
  return success(result.stdout.strip())

def _check_version(repo_ctx, pc_args):
  version_args = []
  for name in ["atleast_version", "max_version", "exact_version"]:
    value = getattr(repo_ctx.attr, name, "")
    if value:
      version_args += ["--{}={}".format(name.replace("_", "-"), value)]
  if not version_args:
    return success(True)

  result = unwrap(repo_ctx).execute(pc_args + version_args)
  if result.return_code != 0:
    version = _installed_version(repo_ctx, pc_args)
    if version.error != None:
      return version
    return _fail(repo_ctx,
                 "Installed version {} doesn't match constraints: {}".format(
                     version.value, " ".join(version_args)),
                 result.stderr)
  else:
    return success(True)

def _cflags(repo_ctx, pc_args):
  result = unwrap(repo_ctx).execute(pc_args + ["--cflags"])
  if result.return_code != 0:
    return _fail(repo_ctx, "Unable to determine cflags", result.stderr)
  stdout = result.stdout
  return success([arg for arg in stdout.strip().split(" ") if arg])

def _linkopts(repo_ctx, pc_args):
  result = unwrap(repo_ctx).execute(pc_args + ["--static", "--libs"])
  if result.return_code != 0:
    return _fail(repo_ctx, "Unable to determine linkopts", result.stderr)
  stdout = result.stdout
  args = [arg for arg in stdout.strip().split(" ") if arg]
  # Bazel "linkopts=" must be either switches ("-foo"), variables ("$(FOO)"),
  # or labels ("foo").  We should only get switches from `pkg-config --libs`.
  # However, sometimes it produces "-framework CoreFoundation" or similar,
  # which is *supposed* to be a single switch, but our split heuristic chopped
  # it up.  We recombine non-switch args with their preceeding arg as a repair.
  # We process args in reserve order to keep our loop index unchanged by a pop.
  for i in reversed(range(len(args))):
    # Switches stay put.
    if args[i].startswith("-"):
      continue
    # A non-switch arg should be recombined with the preceding arg.
    non_switch_arg = args.pop(i)
    if i == 0:
      _fail()
    args[i - 1] += " " + non_switch_arg
  return success(args)

def _extract_prefix(flags, prefix):
  stripped, remain = [], []
  for arg in flags:
    if arg.startswith(prefix):
      stripped += [arg[len(prefix):]]
    else:
      remain += [arg]
  return stripped, remain

def _extract_includes(cflags):
  return _extract_prefix(cflags, "-I")

def _extract_defines(cflags):
  return _extract_prefix(cflags, "-D")

def _symlink_directories(repo_ctx, basename, pathnames):
  result = []
  root = unwrap(repo_ctx).path("")
  base = root.get_child(basename)
  rootlen = len(str(base)) - len(basename)  # Include separator length.
  for srcpath in [unwrap(repo_ctx).path(p) for p in pathnames]:
    destpath = base.get_child(str(srcpath).replace('/', '_'))
    unwrap(repo_ctx).symlink(srcpath, destpath)
    result += [str(destpath)[rootlen:]]
  return result

def _parse_cflags(repo_ctx, cflags):
  includes, cflags = _extract_includes(cflags)
  defines, cflags = _extract_defines(cflags)
  if cflags:
    print("In pkg-config module {}, unhandled cflags: {}".format(
          repo_ctx.attr.modname, cflags))

  # We can only reliably point to include directories.
  # `linkopts` leak into runtime for dynamic libraries and
  # should point to the system paths.
  includes = _symlink_directories(repo_ctx, "include", includes)
  return includes, defines

def _include_dir(repo_ctx, pc_args):
  result = unwrap(repo_ctx).execute(pc_args + ["--variable=includedir"])
  if result.return_code != 0:
    return _fail(repo_ctx, "Unable to determine include dir", result.stderr)
  return success(result.stdout.strip())

def _lib_dir(repo_ctx, pc_args):
  result = unwrap(repo_ctx).execute(pc_args + ["--variable=libdir"])
  if result.return_code != 0:
    return _fail(repo_ctx, "Unable to determine lib dir", result.stderr)
  return success(result.stdout.strip())

def setup_pkg_config_package(repo_ctx):
  generate_cps = getattr(repo_ctx.attr, 'generate_cps', False)
  license = getattr(repo_ctx.attr, 'license', "")
  if generate_cps and not license:
    fail("Must provide a license when generating cps")

  pkg_config = _find(repo_ctx)
  if pkg_config.error != None:
    return pkg_config
  pc_args = [pkg_config.value, repo_ctx.attr.modname]
  exists = _exists(repo_ctx, pc_args)
  if exists.error != None:
    return exists
  version = _check_version(repo_ctx, pc_args)
  if version.error != None:
    return version
  cflags = _cflags(repo_ctx, pc_args)
  if cflags.error != None:
    return cflags

  linkopts = _linkopts(repo_ctx, pc_args)
  if linkopts.error != None:
    return linkopts

  includedir = _include_dir(repo_ctx, pc_args)
  if includedir.error != None:
    return includedir

  installed_vers = _installed_version(repo_ctx, pc_args)
  if installed_vers.error != None:
    return installed_vers

  libdir = _lib_dir(repo_ctx, pc_args)
  if libdir.error != None:
    return libdir

  _write_build(repo_ctx, cflags.value, linkopts.value,
               install_content=_cps_build_content.format(package=repo_ctx.attr.modname))

  if generate_cps:
    extension = "dylib" if repo_ctx.os.name == "mac os x" else "so"
    repo_ctx.file('ccd.cps',
                  _cps_content.format(vers=installed_vers.value,
                                      package=repo_ctx.attr.modname,
                                      license=license,
                                      include=includedir.value,
                                      libdir=libdir.value,
                                      extension=extension),
                  False)

  return success(True)

def _impl(repo_ctx):
  error = setup_pkg_config_package(repo_ctx).error
  if error != None:
    fail(error)

pkg_config_package = repository_rule(
    _impl,
    attrs = {
        "modname": attr.string(mandatory = True),
        "atleast_version": attr.string(),
        "max_version": attr.string(),
        "exact_version": attr.string(),
        "generate_cps": attr.bool(),
        "license": attr.string(),
        "build_file_template": attr.label(
            default = Label("@kythe//tools/build_rules/config:BUILD.tpl"),
            single_file = True,
            allow_files = True,
        ),
    },
    local = True,
)
