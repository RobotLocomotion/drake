# -*- python -*-

load("//tools/workspace:os.bzl", "determine_os")
load("//tools/skylark:pathutils.bzl", "join_paths", "basename")

def _get_macos_codename(release):
    """Returns the codename for a given macOS release. For example, it returns
    'high_sierra' for '10.13'.
    """
    codenames = {
        "10.12": "sierra",
        "10.13": "high_sierra",
    }
    if release in codenames:
        return codenames[release]
    else:
        fail("macOS {} is not supported".format(release))

def _find_lib_pathname(repository_ctx, formula_name):
    """Returns the full pathname of the library (.so or .dylib file) of a given
    formula. For instance, `_find_lib_pathname(repository_ctx, "ibex")` returns
    /private/var/tmp/.../external/ibex_homebrew_bottle/lib/libibex.dylib'. If
    there is no library file, raises an error."""
    lib_dirpath = repository_ctx.path("lib")
    lib_prefix = "lib" + formula_name
    # Check if .dylib exists.
    lib_name = lib_prefix + ".dylib"
    lib_path = lib_dirpath.get_child(lib_name)
    if not lib_path.exists:
        # Check if .so exists.
        lib_name = lib_prefix + ".so"
        lib_path = lib_dirpath.get_child(lib_name)
        if not lib_path.exists:
            msg = "No library (.dylib or .so) found for " + formula_name + "."
            fail(msg)
    return str(lib_path)

def _fix_install_names(repository_ctx, formula_name, dep_map):
    """Fixes the install names of a given library. When building bottles,
    Homebrew adds '@@HOMEBREW_PREFIX@@' and substitute this with a proper value
    when a bottle is installed. Since we are consuming a homebrew bottle in our
    own fashion, we need to do this substitution by ourselves.

    This functions does the two things:

     - It changes the shared library identification name of a given library to
       its current pathname.

     - Based on the user-provided `dep_map` parameter, it changes the install
       names starting with `@@HOMEBREW_PREFIX@@` to a path relative to
       `@loader_path`.
    """
    chmod_path = "/bin/chmod"
    install_name_tool_path = repository_ctx.which("install_name_tool")
    otool_path = "/usr/bin/otool"
    lib_pathname = _find_lib_pathname(repository_ctx, formula_name)
    # Bottles have read-only permission, makes it writable to make changes.
    repository_ctx.execute([chmod_path, "+w", lib_pathname])
    # Changes its identification name.
    repository_ctx.execute([install_name_tool_path,
                            "-id",
                            lib_pathname,
                            lib_pathname])
    # Changes its install names.
    result = repository_ctx.execute([otool_path, "-L", lib_pathname])
    for line in result.stdout.split("\n"):
        if "@@HOMEBREW_PREFIX@@" in line:
            install_name_before = line.strip().split(" ")[0]
            install_basename = basename(install_name_before)
            if install_basename in dep_map:
                install_name_after = join_paths("@loader_path",
                                                "..",
                                                "..",
                                                dep_map[install_basename])
                repository_ctx.execute([install_name_tool_path,
                                        "-change",
                                        install_name_before,
                                        install_name_after,
                                        lib_pathname])
    # Switches back to read-only.
    repository_ctx.execute([chmod_path, "-w", lib_pathname])

def _impl(repository_ctx):
    name = repository_ctx.attr.name
    formula_name = repository_ctx.attr.formula_name
    version = repository_ctx.attr.version
    root_url = repository_ctx.attr.root_url
    sha256s = repository_ctx.attr.sha256s
    build_file = repository_ctx.attr.build_file
    dep_map = repository_ctx.attr.dep_map

    # Download and Extract.
    os_result = determine_os(repository_ctx)
    if not os_result.is_macos:
        fail("Not on macOS.")
    macos_release = os_result.macos_release
    if macos_release not in sha256s:
        fail("SHA for {} is not provided.".format(macos_release))
    sha256 = sha256s[macos_release]
    url = join_paths(root_url,
                     "{0}-{1}.{2}.bottle.tar.gz"
                     .format(formula_name,
                             version,
                             _get_macos_codename(macos_release)))
    strip_prefix = join_paths(formula_name, version)
    repository_ctx.download_and_extract(url,
                                        sha256 = sha256,
                                        type = 'tar.gz',
                                        stripPrefix = strip_prefix)
    _fix_install_names(repository_ctx, formula_name, dep_map)
    repository_ctx.symlink(build_file, "BUILD.bazel")

"""A repository rule that downloads and unpacks a homebrew formula.
"""

new_homebrew_bottle_archive = repository_rule(
    attrs = {
        "formula_name": attr.string(
            doc = """
          Name of homebrew formula.
          """,
            mandatory = True,
        ),
        "version": attr.string(
            doc = """
          Version of homebrew formula.
          """,
            mandatory = True,
        ),
        "root_url": attr.string(
            doc = """
            `root_url` of homebrew formula.
            """,
            mandatory = True,
        ),
        "sha256s": attr.string_dict(
            doc = """
            A dictionary which maps a macOS release (i.e. "10.12") to the
            checksum of the corresponding bottle file.
            """,
            mandatory = True,
        ),
        "build_file": attr.label(
            doc = """
            Label for BUILD.bazel file to add into the repository. This should
            contain the rules that expose the archive contents for consumers.
            """,
            mandatory = True,
            single_file = True,
            allow_files = True,
        ),
        "dep_map": attr.string_dict(
            doc = """
            A dictionary which maps a name of a dependent shared library
            (i.e. "libibex.dylib") to its relative location from
            'bazel-drake-distro/external' folder
            (i.e. "ibex_homebrew_bottle/lib/libibex.dylib").
            See _fix_install_names for details.
            """,
        ),
    },
    implementation = _impl,
)
