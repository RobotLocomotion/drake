# -*- python -*-

InstallInfo = provider()

#==============================================================================
#BEGIN internal helpers

#------------------------------------------------------------------------------
def _join_paths(*args):
    """Join paths without duplicating separators.

    This is roughly equivalent to Python's `os.path.join`.

    Args:
        \*args (:obj:`list` of :obj:`str`): Path components to be joined.

    Returns:
        :obj:`str`: The concatenation of the input path components.
    """
    result = ""

    for part in args:
        if len(part) == 0:
            continue

        result += part
        if result[-1] != "/":
            result += "/"

    return result[:-1]

#------------------------------------------------------------------------------
# Remove prefix from path.
def ___remove_prefix(path, prefix):
    # If the prefix has more parts than the path, failure is certain.
    if len(prefix) > len(path):
        return None

    # Iterate over components to determine if a match exists.
    for n in range(len(prefix)):
        if prefix[n] == path[n]:
            continue
        elif prefix[n] == "*":
            continue
        else:
            return None

    return "/".join(path[len(prefix):])

def __remove_prefix(path, prefix):
    # Ignore trailing empty element (happens if prefix string ends with "/").
    if len(prefix[-1]) == 0:
        prefix = prefix[:-1]

    # If the prefix has more parts than the path, failure is certain. (We also
    # need at least one component of the path left over so the stripped path is
    # not empty.)
    if len(prefix) > (len(path) - 1):
        return None

    # Iterate over components to determine if a match exists.
    for n in range(len(prefix)):
        # Same path components match.
        if prefix[n] == path[n]:
            continue

        # Single-glob matches any (one) path component.
        elif prefix[n] == "*":
            continue

        # Mulit-glob matches one or more components.
        elif prefix[n] == "**":
            # If multi-glob is at the end of the prefix, return the last path
            # component.
            if n + 1 == len(prefix):
                return path[-1]

            # Otherwise, the most components the multi-glob can match is the
            # remaining components (len(prefix) - n - 1; the 1 is the current
            # prefix component) less one (since we need to keep at least one
            # component of the path).
            k = len(path) - (len(prefix) - n - 2)

            # Try to complete the match, iterating (backwards) over the number
            # of components that the multi-glob might match.
            for t in reversed(range(n, k)):
                x = ___remove_prefix(path[t:], prefix[n + 1:])
                if x != None:
                    return x

            # Multi-glob failed to match.
            return None

        else:
            # Components did not match.
            return None

    return "/".join(path[len(prefix):])

def _remove_prefix(path, prefix):
    """Remove prefix from path.

    This attempts to remove the specified prefix from the specified path. The
    prefix may contain the globs ``*`` or ``**``, which match one or many
    path components, respectively. Matching is greedy. Globs may only be
    matched against complete path components (e.g. ``a/*/`` is okay, but
    ``a*/`` is not treated as a glob and will be matched literally). Due to
    Skylark limitations, at most one ``**`` may be matched.

    Args:
        path (:obj:`str`) The path to modify.
        prefix (:obj:`str`) The prefix to remove.

    Returns:
        :obj:`str`: The path with the prefix removed if successful, or None if
        the prefix does not match the path.
    """
    return __remove_prefix(path.split("/"), prefix.split("/"))

#------------------------------------------------------------------------------
def _is_drake_label(x):
    root = x.workspace_root
    if root == "":
        x.package.startswith("drake") or fail("Unknown '%s'" % x.package)
        return True
    else:
        root.startswith("external") or fail("Unknown '%s'" % root)
        return False

#------------------------------------------------------------------------------
def _output_path(ctx, input_file, strip_prefix):
    """Compute output path (without destination prefix) for install action.

    This computes the adjusted output path for an input file. Specifically, it
    a) determines the path relative to the invoking context (which is usually,
    but not always, the same as the path as specified by the user when the file
    was mentioned in a rule), without Bazel's various possible extras, and b)
    to optionally removes prefixes from this path. When removing prefixes, the
    first matching prefix is removed.

    For example::

        install_files(
            dest = "docs",
            files = ["foo/bar.txt"],
            strip_prefix = ["foo/"],
            ...)

    The :obj:`File`'s path components will have various Bazel bits added. Our
    first step is to recover the input path, ``foo/bar.txt``. Then we remove
    the prefix ``foo``, giving a path of ``bar.txt``, which will become
    ``docs/bar.txt`` when the install destination is added.

    Args:
        input_file (:obj:`File`): Artifact to be installed.
        strip_prefix (:obj:`list` of :obj:`str`): List of prefixes to strip
            from the input path before prepending the destination.

    Returns:
        :obj:`str`: The install destination path for the file.
    """

    # Determine base path of invoking context.
    package_root = _join_paths(ctx.label.workspace_root, ctx.label.package)

    # Determine effective path by removing path of invoking context and any
    # Bazel output-files path.
    input_path = input_file.path
    if input_file.is_source:
        input_path = _remove_prefix(input_path, package_root)
    else:
        out_root = _join_paths("bazel-out/*/*", package_root)
        input_path = _remove_prefix(input_path, out_root)

    # Deal with possible case of file outside the package root.
    if input_path == None:
        print("%s installing file %s which is not in current package"
              % (package_root, input_file.path))
        return input_file.basename

    # Possibly remove prefixes.
    for p in strip_prefix:
        output_path = _remove_prefix(input_path, p)
        if output_path != None:
            return output_path

    return input_path

#------------------------------------------------------------------------------
def _install_actions(ctx, file_labels, dests, strip_prefixes = []):
    """Compute install actions for files.

    This takes a list of labels (targets or files) and computes the install
    actions for the files owned by each label.

    Args:
        file_labels (:obj:`list` of :obj:`Label`): labels to install.
        dests (:obj:`str` or :obj:`dict` of :obj:`str` to :obj:`str`):
            Install destination. A :obj:`dict` may be given to supply a mapping
            of file extension to destination path. The :obj:`dict` must have an
            entry with the key ``None`` that is used as the default when there
            is no entry for the specific extension.
        strip_prefixes (:obj:`list` of :obj:`str` or :obj:`dict` of :obj:`list`
            of :obj:`str` to :obj:`str`): List of prefixes to strip from the
            input path before prepending the destination. A :obj:`dict` may be
            given to supply a mapping of file extension to list of prefixes to
            strip. The :obj:`dict` must have an entry with the key ``None``
            that is used as the default when there is no entry for the specific
            extension.

    Returns:
        :obj:`list`: A list of install actions.
    """
    actions = []

    # Iterate over files. We expect a list of labels, which will have a 'files'
    # attribute that is a list of File artifacts. Thus this two-level loop.
    for f in file_labels:
        for a in f.files:
            if type(dests) == "dict":
                dest = dests.get(a.extension, dests[None])
            else:
                dest = dests

            if type(strip_prefixes) == "dict":
                strip_prefix = strip_prefixes.get(a.extension,
                                                  strip_prefixes[None])
            else:
                strip_prefix = strip_prefixes

            p = _output_path(ctx, a, strip_prefix)
            actions.append(struct(src = a, dst = _join_paths(dest, p)))

    return actions

#------------------------------------------------------------------------------
# Compute install actions for a cc_library or cc_binary.
def _install_cc_actions(ctx, target):
    # Compute actions for target artifacts.
    dests = {
        "a": ctx.attr.archive_dest,
        "so": ctx.attr.library_dest,
        None: ctx.attr.runtime_dest,
    }
    strip_prefixes = {
        "a": ctx.attr.archive_strip_prefix,
        "so": ctx.attr.library_strip_prefix,
        None: ctx.attr.runtime_strip_prefix,
    }
    actions = _install_actions(ctx, [target], dests, strip_prefixes)

    # Compute actions for guessed headers.
    if ctx.attr.guess_hdrs != "NONE":
        hdrs = []

        if ctx.attr.guess_hdrs == "EVERYTHING":
            hdrs = target.cc.transitive_headers

        elif ctx.attr.guess_hdrs == "WORKSPACE":
            hdrs = [h for h in target.cc.transitive_headers if
                    target.label.workspace_root == h.owner.workspace_root]

        elif ctx.attr.guess_hdrs == "PACKAGE":
            hdrs = [h for h in target.cc.transitive_headers if
                    target.label.workspace_root == h.owner.workspace_root and
                    target.label.package == h.owner.package]

        else:
            msg_fmt = "'install' given unknown 'guess_hdrs' value '%s'"
            fail(msg_fmt % ctx.attr.guess_hdrs, ctx.attr.guess_hdrs)

        actions += _install_actions(ctx, [struct(files=hdrs)],
                                    ctx.attr.hdr_dest,
                                    ctx.attr.hdr_strip_prefix)

    # Return computed actions.
    return actions

#------------------------------------------------------------------------------
# Compute install actions for a java_library or java_binary.
def _install_java_actions(ctx, target):
    dests = {
        "jar": ctx.attr.java_dest,
        None: ctx.attr.runtime_dest,
    }
    strip_prefixes = {
        "jar": ctx.attr.java_strip_prefix,
        None: ctx.attr.runtime_strip_prefix,
    }

    return _install_actions(ctx, [target], dests, strip_prefixes)

#------------------------------------------------------------------------------
# Compute install actions for a py_library or py_binary.
# TODO(jamiesnape): Install native shared libraries that the target may use.
def _install_py_actions(ctx, target):
    return _install_actions(ctx, [target], ctx.attr.py_dest,
                            ctx.attr.py_strip_prefix)

#------------------------------------------------------------------------------
# Generate install code for an install action.
def _install_code(action):
    return "install(%r, %r)" % (action.src.short_path, action.dst)

#END internal helpers
#==============================================================================
#BEGIN rules

#------------------------------------------------------------------------------
# Generate information to install "stuff". "Stuff" can be library or binary
# targets, headers, or documentation files.
def _install_impl(ctx):
    actions = []

    # Check for missing license files.
    non_drake_labels = [
        x.label for x in ctx.attr.targets + ctx.attr.hdrs
        if not _is_drake_label(x.label)]
    if non_drake_labels and not ctx.attr.license_docs:
        fail("%s is missing required license_docs= attribute for %s" % (
            ctx.label, non_drake_labels))

    # Collect install actions from dependencies.
    for d in ctx.attr.deps:
        actions += d.install_actions

    # Generate actions for docs and includes.
    actions += _install_actions(ctx, ctx.attr.license_docs, ctx.attr.doc_dest,
                                ctx.attr.doc_strip_prefix)
    actions += _install_actions(ctx, ctx.attr.docs, ctx.attr.doc_dest,
                                ctx.attr.doc_strip_prefix)
    actions += _install_actions(ctx, ctx.attr.hdrs, ctx.attr.hdr_dest,
                                ctx.attr.hdr_strip_prefix)

    for t in ctx.attr.targets:
        # TODO(jwnimmer-tri): Raise an error if a target has testonly=1.
        if hasattr(t, "cc"):
            actions += _install_cc_actions(ctx, t)
        elif hasattr(t, "java"):
            actions += _install_java_actions(ctx, t)
        elif hasattr(t, "py"):
            actions += _install_py_actions(ctx, t)

    # Generate code for install actions.
    script_actions = [_install_code(a) for a in actions]

    # Generate install script.
    # TODO(mwoehlke-kitware): Figure out a better way to generate this and run
    # it via Python than `#!/usr/bin/env python`?
    ctx.template_action(
        template = ctx.executable.install_script_template,
        output = ctx.outputs.executable,
        substitutions = {"<<actions>>": "\n    ".join(script_actions)})

    # Return actions.
    files = ctx.runfiles(files = [a.src for a in actions])
    return InstallInfo(install_actions = actions, runfiles = files)

install = rule(
    attrs = {
        "deps": attr.label_list(providers = ["install_actions"]),
        "docs": attr.label_list(allow_files = True),
        "doc_dest": attr.string(default = "share/doc"),
        "doc_strip_prefix": attr.string_list(),
        "license_docs": attr.label_list(allow_files = True),
        "hdrs": attr.label_list(allow_files = True),
        "hdr_dest": attr.string(default = "include"),
        "hdr_strip_prefix": attr.string_list(),
        "guess_hdrs": attr.string(default = "NONE"),
        "targets": attr.label_list(),
        "archive_dest": attr.string(default = "lib"),
        "archive_strip_prefix": attr.string_list(),
        "library_dest": attr.string(default = "lib"),
        "library_strip_prefix": attr.string_list(),
        "runtime_dest": attr.string(default = "bin"),
        "runtime_strip_prefix": attr.string_list(),
        "java_dest": attr.string(default = "share/java"),
        "java_strip_prefix": attr.string_list(),
        "py_dest": attr.string(default = "lib/python2.7/site_packages"),
        "py_strip_prefix": attr.string_list(),
        "install_script_template": attr.label(
            allow_files = True,
            executable = True,
            cfg = "target",
            default = Label("//tools:install.py.in"),
        ),
    },
    executable = True,
    implementation = _install_impl,
)

"""Generate installation information for various artifacts.

This generates installation information for various artifacts, including
documentation and header files, and targets (e.g. ``cc_binary``). By default,
the path of any files is included in the install destination.
See :rule:`install_files` for details.

Note:
    By default, headers to be installed must be explicitly listed. This is to
    work around an issue where Bazel does not appear to provide any mechanism
    to obtain the public headers of a target at rule instantiation. The
    ``guess_hdrs`` parameter may be used to tell ``install`` to guess at what
    headers will be installed. Possible values are:

    * ``"NONE"``: Only install headers which are explicitly listed by ``hdrs``.
    * ``PACKAGE``:  For each target, install those headers which are used by
      the target and owned by a target in the same package.
    * ``WORKSPACE``: For each target, install those headers which are used by
      the target and owned by a target in the same workspace.
    * ``EVERYTHING``: Install all headers used by the target.

    The headers considered are *all* headers transitively used by the target.
    Any option other than ``NONE`` is also likely to install private headers.

Args:
    deps: List of other install rules that this rule should include.
    docs: List of documentation files to install.
    doc_dest: Destination for documentation files (default = "share/doc").
    doc_strip_prefix: List of prefixes to remove from documentation paths.
    license_docs: List of license files to install (uses doc_dest).
    guess_hdrs: See note.
    hdrs: List of header files to install.
    hdr_dest: Destination for header files (default = "include").
    hdr_strip_prefix: List of prefixes to remove from header paths.
    targets: List of targets to install.
    archive_dest: Destination for static library targets (default = "lib").
    archive_strip_prefix: List of prefixes to remove from static library paths.
    library_dest: Destination for shared library targets (default = "lib").
    library_strip_prefix: List of prefixes to remove from shared library paths.
    runtime_dest: Destination for executable targets (default = "bin").
    runtime_strip_prefix: List of prefixes to remove from executable paths.
    java_dest: Destination for Java library targets (default = "share/java").
    java_strip_prefix: List of prefixes to remove from Java library paths.
    py_dest: Destination for Python targets
        (default = "lib/python2.7/site_packages").
    py_strip_prefix: List of prefixes to remove from Python paths.
"""

#------------------------------------------------------------------------------
# Generate information to install files to specified destination.
def _install_files_impl(ctx):
    # Get path components.
    dest = ctx.attr.dest
    strip_prefix = ctx.attr.strip_prefix

    # Generate actions.
    actions = _install_actions(ctx, ctx.attr.files, dest, strip_prefix)

    # Return computed actions.
    return InstallInfo(install_actions = actions)

install_files = rule(
    attrs = {
        "dest": attr.string(mandatory = True),
        "files": attr.label_list(allow_files = True),
        "strip_prefix": attr.string_list(),
    },
    implementation = _install_files_impl,
)

"""Generate installation information for files.

This generates installation information for a list of files. By default, any
path portion of the file as named is included in the install destination. For
example::

    install_files(
        dest = "docs",
        files = ["foo/bar.txt"],
        ...)

This will install ``bar.txt`` to the destination ``docs/foo``.

When this behavior is undesired, the ``strip_prefix`` parameter may be used to
specify a list of prefixes to be removed from input file paths before computing
the destination path. Stripping is not recursive; the first matching prefix
will be stripped. Prefixes support the single-glob (``*``) to match any single
path component, or the multi-glob (``**``) to match any number of path
components. Multi-glob matching is greedy. Globs may only be matched against
complete path components (e.g. ``a/*/`` is okay, but ``a*/`` is not treated as
a glob and will be matched literally). Due to Skylark limitations, at most one
``**`` may be matched.

Args:
    dest: Destination for files.
    files: List of files to install.
    strip_prefix: List of prefixes to remove from input paths.
"""

#END rules
#==============================================================================
#BEGIN macros

#------------------------------------------------------------------------------
def exports_create_cps_scripts(packages):
    """Export scripts that create CPS files to other packages.

    Args:
        packages (:obj:`list` of :obj:`str`): Bazel package names.
    """

    for package in packages:
        native.exports_files(
            ["{}-create-cps.py".format(package)],
            visibility = ["@{}//:__pkg__".format(package)],
        )

#------------------------------------------------------------------------------
def cmake_config(package, script=None, version_file=None, deps=[]):
    """Create CMake package configuration and package version files via an
    intermediate CPS file.

    Args:
        package (:obj:`str`): CMake package name.
        script (:obj:`Label`): Script that creates the intermediate CPS file.
        version_file (:obj:`str`): File that the script will search to
            determine the version of the package.
    """

    if script and version_file:
        native.py_binary(
            name = "create-cps",
            srcs = [script],
            main = script,
            visibility = ["//visibility:private"],
            deps = ["@drake//tools:cpsutils"],
        )

        cps_file_name = "{}.cps".format(package)

        native.genrule(
            name = "cps",
            srcs = [version_file] + deps,
            outs = [cps_file_name],
            cmd = "$(location :create-cps) $(SRCS) > \"$@\"",
            tools = [":create-cps"],
            visibility = ["//visibility:public"],
        )
    else:
        cps_file_name = "@drake//tools:{}.cps".format(package)

    config_file_name = "{}Config.cmake".format(package)

    native.genrule(
        name = "cmake_exports",
        srcs = [cps_file_name],
        outs = [config_file_name],
        cmd = "$(location @pycps//:cps2cmake_executable) \"$<\" > \"$@\"",
        tools = ["@pycps//:cps2cmake_executable"],
        visibility = ["//visibility:private"],
    )

    config_version_file_name = "{}ConfigVersion.cmake".format(package)

    native.genrule(
        name = "cmake_package_version",
        srcs = [cps_file_name],
        outs = [config_version_file_name],
        cmd = "$(location @pycps//:cps2cmake_executable) --version-check \"$<\" > \"$@\"",
        tools = ["@pycps//:cps2cmake_executable"],
        visibility = ["//visibility:private"],
    )

#------------------------------------------------------------------------------
def install_cmake_config(package, versioned=True):
    """Generate installation information for CMake package configuration and
    package version files. The rule name is always ``:install_cmake_config``.

    Args:
        package (:obj:`str`): CMake package name.
        versioned (:obj:`bool`): True if a version file should be installed.
    """
    cmake_config_dest = "lib/cmake/{}".format(package.lower())
    cmake_config_files = ["{}Config.cmake".format(package)]

    if versioned:
        cmake_config_files += ["{}ConfigVersion.cmake".format(package)]

    install_files(
        name = "install_cmake_config",
        dest = cmake_config_dest,
        files = cmake_config_files,
        visibility = ["//visibility:private"],
    )

#END macros
