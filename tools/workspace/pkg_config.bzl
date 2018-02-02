# -*- python -*-

_DEFAULT_TEMPLATE = Label("@drake//tools/workspace:pkg_config.BUILD.tpl")

_DEFAULT_STATIC = False

def _run_pkg_config(repository_ctx, command_line, pkg_config_paths):
    """Run command_line with PKG_CONFIG_PATH = pkg_config_paths and return its
    tokenized output."""
    pkg_config_path = ":".join(pkg_config_paths)
    result = repository_ctx.execute(command_line,
                                    environment = {
                                        "PKG_CONFIG_PATH": pkg_config_path,
                                    })
    if result.return_code != 0:
        return struct(error = "error {} from {}: {}{}".format(
            result.return_code, command_line, result.stdout, result.stderr))
    tokens = [x for x in result.stdout.strip().split(" ") if x]
    return struct(tokens = tokens, error = None)

def setup_pkg_config_repository(repository_ctx):
    """This is the macro form of the pkg_config_repository() rule below.
    Refer to that rule's API documentation for details.

    This flavor of this rule is intended to be called by other repository_rule
    implementation functions.  The pkg_config_repository flavor of this rule is
    intended to be called directly from the WORKSPACE file, or from a macro
    that was called by the WORKSPACE file.
    """
    # First locate pkg-config on the $PATH.
    tool_path = repository_ctx.which("pkg-config")
    if not tool_path:
        return struct(error = "Could not find pkg-config on PATH={}".format(
            repository_ctx.os.environ["PATH"]))
    args = [tool_path, repository_ctx.attr.modname]

    pkg_config_paths = getattr(repository_ctx.attr,
                               "pkg_config_paths",
                               [])

    # Check if we can find the required *.pc file of any version.
    result = _run_pkg_config(repository_ctx, args, pkg_config_paths)
    if result.error != None:
        return result

    # If we have a minimum version, enforce that.
    atleast_version = getattr(repository_ctx.attr, "atleast_version", "")
    if atleast_version:
        result = _run_pkg_config(repository_ctx, args + [
            "--atleast-version", atleast_version], pkg_config_paths)
        if result.error != None:
            return struct(error = result.error + "during version check")

    # Determine linkopts.
    static = getattr(repository_ctx.attr, "static", _DEFAULT_STATIC)
    libs_args = args + ["--libs"]
    if static:
        libs_args = libs_args + ["--static"]
    result = _run_pkg_config(repository_ctx, libs_args, pkg_config_paths)
    if result.error != None:
        return result
    linkopts = result.tokens
    # Bazel "linkopts=" must be either switches ("-foo"), variables ("$(FOO)"),
    # or labels ("foo").  We should only get switches from `pkg-config --libs`.
    # However, sometimes it produces "-framework CoreFoundation" or similar,
    # which is *supposed* to be a single switch, but our split heuristic
    # chopped it up.  We recombine non-switch args with their preceeding arg as
    # a repair.  We process args in reserve order to keep our loop index
    # unchanged by a pop.
    for i in reversed(range(len(linkopts))):
        linkopt = linkopts[i]
        # Absolute system paths to *.so files get turned into -l instead.
        # This fixup is only implemented for Ubuntu (not macOS) so far.
        if linkopt.endswith(".so"):
            possible_libdirs = [
                "/usr/lib",
                "/usr/lib/x86_64-linux-gnu",
            ]
            for dir in possible_libdirs:
                prefix = dir + "/lib"
                if linkopt.startswith(prefix):
                    name = linkopt[len(prefix):-len(".so")]
                    if "/" not in name:
                        linkopt = "-l" + name
                        linkopts[i] = linkopt
                        break
        # Add `-Wl,-rpath <path>` for `-L<path>`.
        # See https://github.com/RobotLocomotion/drake/issues/7387#issuecomment-359952616  # noqa
        if linkopt.startswith("-L"):
            linkopts[i] = "-Wl,-rpath " + linkopt[2:] + " " + linkopt
            continue
        # Switches stay put.
        if linkopt.startswith("-"):
            continue
        # A non-switch arg should be recombined with the preceding arg.
        non_switch_arg = linkopts.pop(i)
        if i == 0:
            return struct(error = "malformed linkopts: " + repr(linkopts))
        linkopts[i - 1] += " " + non_switch_arg

    # Determine cflags; we'll split into includes and defines in a moment.
    result = _run_pkg_config(repository_ctx,
                             args + ["--cflags"],
                             pkg_config_paths)
    if result.error != None:
        return result
    cflags = result.tokens
    # Placate whiny compilers.
    if "-pthread" in cflags and "-pthread" in linkopts:
        cflags.remove("-pthread")

    # Split cflags into includes and defines.  The -I paths from pkg-config
    # will be absolute paths; we'll make them relative in a moment.
    absolute_includes = []
    defines = []
    unknown_cflags = []
    # We process in reserve order to keep our loop index unchanged by a pop.
    for cflag in cflags:
        if cflag.startswith("-I"):
            absolute_includes += [cflag[2:]]
        elif cflag.startswith("-D"):
            defines += [cflag[2:]]
        elif cflag in [
                "-frounding-math",
                "-ffloat-store",
                "-msse",
                "-msse2",
                "-msse3",
                "-msse4",
                "-mfpmath"]:
            # We know these are okay to ignore.
            pass
        else:
            unknown_cflags += [cflag]
    if unknown_cflags:
        print("pkg-config of {} returned flags that we will ignore: {}".format(
            repository_ctx.attr.modname, unknown_cflags))

    # Symlink the absolute include paths into our repository, to obtain
    # relative paths for them as required by cc_library's attributes.
    includes = []
    hdrs_path = repository_ctx.path("include")
    for item in absolute_includes:
        symlink_dest = item.replace('/', '_')
        repository_ctx.symlink(
            repository_ctx.path(item),
            hdrs_path.get_child(symlink_dest))
        includes += ["include/" + symlink_dest]
    hdrs_prologue = "glob([\"include/**\"]) + "

    # Write out the BUILD file.
    substitutions = {
        "%{name}": repr(
            repository_ctx.name),
        "%{srcs}": repr(
            getattr(repository_ctx.attr, "extra_srcs", [])),
        "%{hdrs}": (
            hdrs_prologue + repr(
                getattr(repository_ctx.attr, "extra_hdrs", []))),
        "%{copts}": repr(
            getattr(repository_ctx.attr, "extra_copts", [])),
        "%{defines}": repr(
            defines + getattr(repository_ctx.attr, "extra_defines", [])),
        "%{includes}": repr(
            includes + getattr(repository_ctx.attr, "extra_includes", [])),
        "%{linkopts}": repr(
            linkopts + getattr(repository_ctx.attr, "extra_linkopts", [])),
        "%{deps}": repr(
            getattr(repository_ctx.attr, "extra_deps", [])),
    }
    template = getattr(
        repository_ctx.attr, "build_file_template", _DEFAULT_TEMPLATE)
    repository_ctx.template("BUILD", template, substitutions)

    return struct(value = True, error = None)

def _impl(repository_ctx):
    result = setup_pkg_config_repository(repository_ctx)
    if result.error != None:
        fail("Unable to complete pkg-config setup for @{} repository: {}".
             format(repository_ctx.name, result.error))

pkg_config_repository = repository_rule(
    attrs = {
        "modname": attr.string(mandatory = True),
        "atleast_version": attr.string(),
        "static": attr.bool(default = _DEFAULT_STATIC),
        "build_file_template": attr.label(
            default = _DEFAULT_TEMPLATE,
            single_file = True,
            allow_files = True,
        ),
        "extra_srcs": attr.string_list(),
        "extra_hdrs": attr.string_list(),
        "extra_copts": attr.string_list(),
        "extra_defines": attr.string_list(),
        "extra_includes": attr.string_list(),
        "extra_linkopts": attr.string_list(),
        "extra_deps": attr.string_list(),
        "pkg_config_paths": attr.string_list(),
    },
    environ = [
        "PATH",
    ],
    local = True,
    implementation = _impl,
)

"""Creates a repository that contains a single library target, based on the
results of invoking pkg-config.

The pkg_config_repository flavor of this rule is intended to be called directly
from the WORKSPACE file, or from a macro that was called by the WORKSPACE file.
The setup_pkg_config_repository flavor of this rule is intended to be called by
other repository_rule implementation functions.

Example:
    WORKSPACE:
        load("@drake//tools/workspace:pkg_config.bzl", "pkg_config_repository")
        pkg_config_repository(
            name = "foo",
            modname = "foo-2.0",
        )

    BUILD:
        cc_library(
            name = "foobar",
            deps = ["@foo"],
            srcs = ["bar.cc"],
        )

Args:
    name: A unique name for this rule.
    modname: The library name as known to pkg-config.
    atleast_version: (Optional) The --atleast-version to pkg-config.
    static: (Optional) Add linkopts for static linking to the library target.
    build_file_template: (Optional) (Advanced) Override the BUILD template.
    extra_srcs: (Optional) Extra items to add to the library target.
    extra_hdrs: (Optional) Extra items to add to the library target.
    extra_copts: (Optional) Extra items to add to the library target.
    extra_defines: (Optional) Extra items to add to the library target.
    extra_includes: (Optional) Extra items to add to the library target.
    extra_linkopts: (Optional) Extra items to add to the library target.
    extra_deps: (Optional) Extra items to add to the library target.
    pkg_config_paths: (Optional) Paths to find pkg-config files (.pc). Note
                      that we ignore the enviornment variable PKG_CONFIG_PATH
                      set by the user.
"""
