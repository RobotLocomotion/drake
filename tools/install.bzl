# -*- python -*-

load("@drake//tools:pathutils.bzl", "dirname", "output_path", "join_paths")

InstallInfo = provider()

#==============================================================================
#BEGIN internal helpers

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
def _output_path(ctx, input_file, strip_prefix = [], warn_foreign = True):
    """Compute output path (without destination prefix) for install action.

    This computes the adjusted output path for an input file. It is the same as
    :func:`output_path`, but additionally handles files outside the current
    package when :func:`install` or :func:`install_files` is invoked with
    non-empty ``allowed_externals``.
    """

    # Try the current package first
    path = output_path(ctx, input_file, strip_prefix)
    if path != None:
        return path

    # If we were unable to resolve a path, the file must be "foreign", so try
    # to resolve against the list of allowed externals.
    if path == None and hasattr(ctx.attr, "allowed_externals"):
        for x in ctx.attr.allowed_externals:
            package_root = join_paths(x.label.workspace_root, x.label.package)
            path = output_path(ctx, input_file, strip_prefix, package_root)
            if path != None:
                return path

    # If we get here, we were not able to resolve the path; give up, and print
    # a warning about installing the "foreign" file.
    if warn_foreign:
        print("%s installing file %s which is not in current package"
              % (ctx.label, input_file.path))
    return input_file.basename

#------------------------------------------------------------------------------
def _guess_files(target, candidates, scope, attr_name):
    if scope == "EVERYTHING":
        return candidates

    elif scope == "WORKSPACE":
        return [f for f in candidates if
                target.label.workspace_root == f.owner.workspace_root]

    elif scope == "PACKAGE":
        return [f for f in candidates if
                target.label.workspace_root == f.owner.workspace_root and
                target.label.package == f.owner.package]

    else:
        msg_fmt = "'install' given unknown '%s' value '%s'"
        fail(msg_fmt % (attr_name, scope), scope)

#------------------------------------------------------------------------------
def _install_action(ctx, artifact, dests, strip_prefixes = [], rename = {}):
    """Compute install action for a single file.

    This takes a single file artifact and returns the appropriate install
    action for the file. The parameters are the same as for
    :func:`_install_action`.
    """
    if type(dests) == "dict":
        dest = dests.get(artifact.extension, dests[None])
    else:
        dest = dests

    if type(strip_prefixes) == "dict":
        strip_prefix = strip_prefixes.get(artifact.extension,
                                          strip_prefixes[None])
    else:
        strip_prefix = strip_prefixes

    file_dest = join_paths(dest, _output_path(ctx, artifact, strip_prefix))
    if file_dest in rename:
        renamed = rename[file_dest]
        file_dest = join_paths(dirname(file_dest), renamed)

    return struct(src = artifact, dst = file_dest)

#------------------------------------------------------------------------------
def _install_actions(ctx, file_labels, dests, strip_prefixes = [],
                     excluded_files = [], rename = {}):
    """Compute install actions for files.

    This takes a list of labels (targets or files) and computes the install
    actions for the files owned by each label.

    Args:
        file_labels (:obj:`list` of :obj:`Label`): List of labels to install.
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
        excluded_files (:obj:`list` of :obj:`str`): List of files to exclude
            from installation.

    Returns:
        :obj:`list`: A list of install actions.
    """
    actions = []

    # Iterate over files. We expect a list of labels, which will have a 'files'
    # attribute that is a list of file artifacts. Thus this two-level loop.
    for f in file_labels:
        for a in f.files:
            # TODO(mwoehlke-kitware) refactor this to separate computing the
            # original relative path and the path with prefix(es) stripped,
            # then use the original relative path for both exclusions and
            # renaming.
            if _output_path(ctx, a, warn_foreign = False) in excluded_files:
                continue

            actions.append(
                _install_action(ctx, a, dests, strip_prefixes, rename))

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
    actions = _install_actions(ctx, [target], dests, strip_prefixes,
                               rename = ctx.attr.rename)

    # Compute actions for guessed resource files.
    if ctx.attr.guess_data != "NONE":
        data = [f for f in target.data_runfiles.files if f.is_source]
        data = _guess_files(target, data, ctx.attr.guess_data, 'guess_data')
        actions += _install_actions(ctx, [struct(files = data)],
                                    ctx.attr.data_dest,
                                    ctx.attr.data_strip_prefix,
                                    ctx.attr.guess_data_exclude,
                                    rename = ctx.attr.rename)

    # Compute actions for guessed headers.
    if ctx.attr.guess_hdrs != "NONE":
        hdrs = _guess_files(target, target.cc.transitive_headers,
                            ctx.attr.guess_hdrs, 'guess_hdrs')
        actions += _install_actions(ctx, [struct(files = hdrs)],
                                    ctx.attr.hdr_dest,
                                    ctx.attr.hdr_strip_prefix,
                                    ctx.attr.guess_hdrs_exclude,
                                    rename = ctx.attr.rename)

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

    return _install_actions(ctx, [target], dests, strip_prefixes,
                            rename = ctx.attr.rename)

#------------------------------------------------------------------------------
# Compute install actions for a py_library or py_binary.
# TODO(jamiesnape): Install native shared libraries that the target may use.
def _install_py_actions(ctx, target):
    return _install_actions(ctx, [target], ctx.attr.py_dest,
                            ctx.attr.py_strip_prefix,
                            rename = ctx.attr.rename)

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

    # Generate actions for data, docs and includes.
    actions += _install_actions(ctx, ctx.attr.license_docs, ctx.attr.doc_dest,
                                strip_prefixes = ctx.attr.doc_strip_prefix,
                                rename = ctx.attr.rename)
    actions += _install_actions(ctx, ctx.attr.docs, ctx.attr.doc_dest,
                                strip_prefixes = ctx.attr.doc_strip_prefix,
                                rename = ctx.attr.rename)
    actions += _install_actions(ctx, ctx.attr.data, ctx.attr.data_dest,
                                strip_prefixes = ctx.attr.data_strip_prefix,
                                rename = ctx.attr.rename)
    actions += _install_actions(ctx, ctx.attr.hdrs, ctx.attr.hdr_dest,
                                strip_prefixes = ctx.attr.hdr_strip_prefix,
                                rename = ctx.attr.rename)

    for t in ctx.attr.targets:
        # TODO(jwnimmer-tri): Raise an error if a target has testonly=1.
        if hasattr(t, "cc"):
            actions += _install_cc_actions(ctx, t)
        elif hasattr(t, "java"):
            actions += _install_java_actions(ctx, t)
        elif hasattr(t, "py"):
            actions += _install_py_actions(ctx, t)

    # Generate code for install actions.
    script_actions = []
    installed_files = {}
    for a in actions:
        if a.dst not in installed_files:
            script_actions.append(_install_code(a))
            installed_files[a.dst] = a.src
        elif a.src != installed_files[a.dst]:
            fail("Install conflict detected:\n" +
                 "\n  src1 = " + repr(installed_files[a.dst]) +
                 "\n  src2 = " + repr(a.src) +
                 "\n  dst = " + repr(a.dst))

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

# TODO(mwoehlke-kitware) default guess_data to PACKAGE when we have better
# default destinations.
install = rule(
    attrs = {
        "deps": attr.label_list(providers = ["install_actions"]),
        "docs": attr.label_list(allow_files = True),
        "doc_dest": attr.string(default = "share/doc"),
        "doc_strip_prefix": attr.string_list(),
        "license_docs": attr.label_list(allow_files = True),
        "data": attr.label_list(allow_files = True),
        "data_dest": attr.string(default = "share"),
        "data_strip_prefix": attr.string_list(),
        "guess_data": attr.string(default = "NONE"),
        "guess_data_exclude": attr.string_list(),
        "hdrs": attr.label_list(allow_files = True),
        "hdr_dest": attr.string(default = "include"),
        "hdr_strip_prefix": attr.string_list(),
        "guess_hdrs": attr.string(default = "NONE"),
        "guess_hdrs_exclude": attr.string_list(),
        "targets": attr.label_list(),
        "archive_dest": attr.string(default = "lib"),
        "archive_strip_prefix": attr.string_list(),
        "library_dest": attr.string(default = "lib"),
        "library_strip_prefix": attr.string_list(),
        "runtime_dest": attr.string(default = "bin"),
        "runtime_strip_prefix": attr.string_list(),
        "java_dest": attr.string(default = "share/java"),
        "java_strip_prefix": attr.string_list(),
        "py_dest": attr.string(default = "lib/python2.7/site-packages"),
        "py_strip_prefix": attr.string_list(),
        "rename": attr.string_dict(),
        "allowed_externals": attr.label_list(allow_files = True),
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

Normally, you should not install files or targets from a workspace other than
the one invoking ``install``, and ``install`` will warn if asked to do so. In
cases (e.g. adding install rules to a project that is natively built with
bazel, but does not define an install) where this *is* the right thing to do,
the ``allowed_externals`` argument may be used to specify a list of externals
whose files it is okay to install, which will suppress the warning.

Note:
    By default, headers and resource files to be installed must be explicitly
    listed. This is to work around an issue where Bazel does not appear to
    provide any mechanism to obtain the public headers of a target, nor the
    *direct* data files of a target, at rule instantiation. The ``guess_hdrs``
    and ``guess_data`` parameters may be used to tell ``install`` to guess at
    what headers and/or resource files will be installed. Possible values are:

    * ``"NONE"``: Only install files which are explicitly listed (i.e. by
      ``hdrs`` or ``data``).
    * ``PACKAGE``:  For each target, install those files which are used by the
      target and owned by a target in the same package.
    * ``WORKSPACE``: For each target, install those files which are used by the
      target and owned by a target in the same workspace.
    * ``EVERYTHING``: Install all headers/resources used by the target.

    The headers and resource files considered are *all* headers or resource
    files transitively used by the target. Any option other than ``NONE`` is
    also likely to install private headers, and may install resource files used
    by other targets. In either case, this may result in the same file being
    considered for installation more than once.

    Note also that, because Bazel includes *all* run-time dependencies —
    including e.g. shared libraries — in a target's ``runfiles``, only *source*
    artifacts are considered when guessing resource files.

Args:
    deps: List of other install rules that this rule should include.
    docs: List of documentation files to install.
    doc_dest: Destination for documentation files (default = "share/doc").
    doc_strip_prefix: List of prefixes to remove from documentation paths.
    license_docs: List of license files to install (uses doc_dest).
    guess_data: See note.
    guess_data_exclude: List of resources found by ``guess_data`` to exclude
        from installation.
    data: List of (platform-independent) resource files to install.
    data_dest: Destination for resource files (default = "share").
    data_strip_prefix: List of prefixes to remove from resource paths.
    guess_hdrs: See note.
    guess_hdrs_exclude: List of headers found by ``guess_hdrs`` to exclude from
        installation.
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
        (default = "lib/python2.7/site-packages").
    py_strip_prefix: List of prefixes to remove from Python paths.
    rename: Mapping of install paths to alternate file names, used to rename
      files upon installation.
    allowed_externals: List of external packages whose files may be installed.
"""

#------------------------------------------------------------------------------
# Generate information to install files to specified destination.
def _install_files_impl(ctx):
    # Get path components.
    dest = ctx.attr.dest
    strip_prefix = ctx.attr.strip_prefix

    # Generate actions.
    actions = _install_actions(ctx, ctx.attr.files, dest, strip_prefix,
                               rename = ctx.attr.rename)

    # Return computed actions.
    return InstallInfo(install_actions = actions)

install_files = rule(
    attrs = {
        "dest": attr.string(mandatory = True),
        "files": attr.label_list(allow_files = True),
        "rename": attr.string_dict(),
        "strip_prefix": attr.string_list(),
        "allowed_externals": attr.string_list(),
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

``install_files`` has the same caveats regarding external files as
:func:`install`.

Args:
    dest: Destination for files.
    files: List of files to install.
    strip_prefix: List of prefixes to remove from input paths.
    rename: Mapping of install paths to alternate file names, used to rename
      files upon installation.
    allowed_externals: List of external packages whose files may be installed.
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
def cmake_config(
    package,
    script = None,
    version_file = None,
    cps_file_name = None,
    deps = []
):
    """Create CMake package configuration and package version files via an
    intermediate CPS file.

    Args:
        package (:obj:`str`): CMake package name.
        script (:obj:`Label`): Script that creates the intermediate CPS file.
        version_file (:obj:`str`): File that the script will search to
            determine the version of the package.
    """

    if script and version_file:
        if cps_file_name:
            fail("cps_file_name should not be set if " +
                 "script and version_file are set.")
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
    elif not cps_file_name:
        cps_file_name = "@drake//tools:{}.cps".format(package)

    config_file_name = "{}Config.cmake".format(package)
    executable = "$(location @pycps//:cps2cmake_executable)"

    native.genrule(
        name = "cmake_exports",
        srcs = [cps_file_name],
        outs = [config_file_name],
        cmd = executable + " \"$<\" > \"$@\"",
        tools = ["@pycps//:cps2cmake_executable"],
        visibility = ["//visibility:private"],
    )

    config_version_file_name = "{}ConfigVersion.cmake".format(package)

    native.genrule(
        name = "cmake_package_version",
        srcs = [cps_file_name],
        outs = [config_version_file_name],
        cmd = executable + " --version-check \"$<\" > \"$@\"",
        tools = ["@pycps//:cps2cmake_executable"],
        visibility = ["//visibility:private"],
    )

#------------------------------------------------------------------------------
def install_cmake_config(
    package,
    versioned = True,
    name = "install_cmake_config",
    visibility = ["//visibility:private"]
):
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
        name = name,
        dest = cmake_config_dest,
        files = cmake_config_files,
        visibility = visibility,
    )

#END macros
