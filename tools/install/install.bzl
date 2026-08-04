load("@rules_license//rules:providers.bzl", "LicenseInfo")
load("//tools/skylark:cc.bzl", "CcInfo")
load("//tools/skylark:drake_py.bzl", "drake_py_test")
load("//tools/skylark:java.bzl", "JavaInfo")
load(
    "//tools/skylark:pathutils.bzl",
    "dirname",
    "join_paths",
    "output_path",
)
load("//tools/skylark:py.bzl", "PyInfo", "py_binary")

InstallInfo = provider()

InstalledTestInfo = provider()

#==============================================================================
#BEGIN internal helpers

#------------------------------------------------------------------------------
def _workspace(ctx):
    """Compute name of current workspace."""

    # Check for override.
    if hasattr(ctx.attr, "workspace"):
        if len(ctx.attr.workspace):
            return ctx.attr.workspace

    # Check for meaningful workspace_root (using the apparent repository name
    # for brevity, not the canonical repository name with  "+" symbols).
    workspace = ctx.label.workspace_root.split("/")[-1]
    workspace = workspace.split("+")[-1]
    if len(workspace):
        return workspace

    # If workspace_root is empty, assume we are the root workspace.
    return ctx.workspace_name

def _rename(file_dest, rename):
    """Compute file name if file renamed."""
    if file_dest in rename:
        renamed = rename[file_dest]
        return join_paths(dirname(file_dest), renamed)
    return file_dest

def _depset_to_list(x):
    """Helper function to convert depset to list."""
    iter_list = x.to_list() if type(x) == "depset" else x
    return iter_list

#------------------------------------------------------------------------------

_PY_CC_TOOLCHAIN_TYPE = "@rules_python//python/cc:toolchain_type"

def _python_version(ctx):
    """Returns a string a containing the major.minor version number of the
    current Python toolchain."""
    py_cc_toolchain = ctx.toolchains[_PY_CC_TOOLCHAIN_TYPE].py_cc_toolchain
    version = py_cc_toolchain.python_version
    major_minor = version.split(".")[:2]
    return ".".join(major_minor)

#------------------------------------------------------------------------------
def _output_path(ctx, input_file, strip_prefix = [], ignore_errors = False):
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

    # If we get here, we were not able to resolve the path; give up.
    if not ignore_errors:
        fail("%s installing file %s which is not in current package" %
             (ctx.label, input_file.path))
    return input_file.basename

#------------------------------------------------------------------------------
def _guess_files(
        target,
        candidates,
        scope,
        attr_name,
        allowed_externals = None):
    if scope == "EVERYTHING":
        return candidates

    elif scope == "WORKSPACE":
        return [
            f
            for f in _depset_to_list(candidates)
            if target.label.workspace_root == f.owner.workspace_root
        ]

    elif scope == "PACKAGE":
        return [
            f
            for f in _depset_to_list(candidates)
            if (target.label.workspace_root == f.owner.workspace_root and
                target.label.package == f.owner.package)
        ]

    elif scope == "ALLOWED_EXTERNALS" and allowed_externals != None:
        return [
            f
            for f in _depset_to_list(candidates)
            if f.is_source and any([
                allowed.label.workspace_root == f.owner.workspace_root and
                allowed.label.package == f.owner.package
                for allowed in allowed_externals
            ])
        ]

    else:
        msg_fmt = "'install' given unknown '%s' value '%s'"
        fail(msg_fmt % (attr_name, scope), scope)

#------------------------------------------------------------------------------
def _install_action(
        ctx,
        artifact,
        dests,
        strip_prefixes = [],
        rename = {}):
    """Compute install action for a single file.

    This takes a single file artifact and returns the appropriate install
    action for the file. The parameters are the same as for
    :func:`_install_action`.
    """
    if type(dests) == "dict":
        dest = dests.get(artifact.extension, dests[None])
    else:
        dest = dests

    if "@WORKSPACE@" in dest:
        dest = dest.replace("@WORKSPACE@", _workspace(ctx))
    if "@PYTHON_VERSION@" in dest:
        dest = dest.replace("@PYTHON_VERSION@", _python_version(ctx))

    if type(strip_prefixes) == "dict":
        strip_prefix = strip_prefixes.get(
            artifact.extension,
            strip_prefixes[None],
        )
    else:
        strip_prefix = strip_prefixes

    file_dest = join_paths(
        dest,
        _output_path(ctx, artifact, strip_prefix),
    )
    file_dest = _rename(file_dest, rename)

    return struct(src = artifact, dst = file_dest)

#------------------------------------------------------------------------------
def _install_actions(
        ctx,
        file_labels,
        dests,
        strip_prefixes = [],
        excluded_files = [],
        rename = {}):
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
        for a in _depset_to_list(f.files):
            # TODO(mwoehlke-kitware) refactor this to separate computing the
            # original relative path and the path with prefix(es) stripped,
            # then use the original relative path for both exclusions and
            # renaming.
            if _output_path(ctx, a, ignore_errors = True) in excluded_files:
                continue

            actions.append(
                _install_action(
                    ctx,
                    a,
                    dests,
                    strip_prefixes,
                    rename,
                ),
            )

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
    actions = _install_actions(
        ctx,
        [target],
        dests,
        strip_prefixes,
        rename = ctx.attr.rename,
    )

    # Compute actions for guessed resource files.
    if ctx.attr.guess_data != "NONE":
        data = [
            f
            for f in _depset_to_list(target.data_runfiles.files)
            if f.is_source
        ]
        data = _guess_files(target, data, ctx.attr.guess_data, "guess_data")
        actions += _install_actions(
            ctx,
            [struct(files = data)],
            ctx.attr.data_dest,
            ctx.attr.data_strip_prefix,
            ctx.attr.guess_data_exclude,
            rename = ctx.attr.rename,
        )

    # Compute actions for guessed headers.
    if ctx.attr.guess_hdrs != "NONE":
        hdrs = _guess_files(
            target,
            target[CcInfo].compilation_context.headers,
            ctx.attr.guess_hdrs,
            "guess_hdrs",
            getattr(ctx.attr, "allowed_externals"),
        )
        actions += _install_actions(
            ctx,
            [struct(files = hdrs)],
            ctx.attr.hdr_dest,
            ctx.attr.hdr_strip_prefix,
            ctx.attr.guess_hdrs_exclude,
            rename = ctx.attr.rename,
        )

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
    excluded_files = []
    if target.files_to_run.executable:
        excluded_files = [
            _output_path(
                ctx,
                target.files_to_run.executable,
                ignore_errors = True,
            ),
        ]
    return _install_actions(
        ctx,
        [target],
        dests,
        strip_prefixes,
        excluded_files,
        rename = ctx.attr.rename,
    )

#------------------------------------------------------------------------------
# Compute install actions for a py_library or py_binary.
# TODO(jamiesnape): Install native shared libraries that the target may use.
def _install_py_actions(ctx, target):
    return _install_actions(
        ctx,
        [target],
        ctx.attr.py_dest,
        ctx.attr.py_strip_prefix,
        rename = ctx.attr.rename,
    )

#------------------------------------------------------------------------------
# Compute install actions for a script or an executable.
def _install_runtime_actions(ctx, target):
    return _install_actions(
        ctx,
        [target],
        ctx.attr.runtime_dest,
        ctx.attr.runtime_strip_prefix,
        rename = ctx.attr.rename,
    )

#------------------------------------------------------------------------------
def _install_test_actions(ctx):
    """Compute and return list of install test command lines.

    This computes the install path for the install tests (tests run to verify
    that the project works once installed).

    Returns:
        :obj:`struct`: A list of test actions containing the location of the
        tests files in the source tree and in the install tree.
    """
    test_actions = []

    # For files, we run the file from the build tree.
    for test in ctx.attr.install_tests:
        for f in _depset_to_list(test.files):
            test_actions.append(
                struct(src = f, cmd = f.path),
            )

    return test_actions

#------------------------------------------------------------------------------
# Generate install code for an install action.
def _install_code(action):
    return "install(%r, %r)" % (action.src.short_path, action.dst)

#------------------------------------------------------------------------------
# Generate install code for a java launcher.
def _java_launcher_code(action):
    return "create_java_launcher(%r, %r, %r, %r)" % (
        action.dst,
        action.classpath,
        " ".join(action.jvm_flags),
        action.main_class,
    )

#END internal helpers
#==============================================================================
#BEGIN rules

def install_license(*, doc_dest, licenses, tags = None, **kwargs):
    """Installs the license file texts for the `licenses` that have been
    declared using rules_license.
    """
    _install_license_rule(
        doc_dest = doc_dest,
        licenses = licenses,
        tags = (tags or []) + ["install"],
        **kwargs
    )

def _install_license_impl(ctx):
    install_actions = []
    for license in ctx.attr.licenses:
        file = license[LicenseInfo].license_text
        install_actions.append(struct(
            src = file,
            dst = ctx.attr.doc_dest + "/" + file.basename,
        ))
    return [
        InstallInfo(
            install_actions = install_actions,
            rename = {},
            installed_files = {},
        ),
    ]

_install_license_rule = rule(
    implementation = _install_license_impl,
    attrs = {
        "doc_dest": attr.string(mandatory = True),
        "licenses": attr.label_list(providers = [LicenseInfo]),
    },
)

#------------------------------------------------------------------------------
# Generate information to install "stuff". "Stuff" can be library or binary
# targets, headers, or documentation files.
def _install_impl(ctx):
    actions = []
    installed_tests = []
    rename = dict(ctx.attr.rename)

    # Collect install actions from dependencies.
    for d in ctx.attr.deps:
        actions += d[InstallInfo].install_actions
        rename.update(d[InstallInfo].rename)
        if InstalledTestInfo in d:
            installed_tests += d[InstalledTestInfo].tests

    # Generate actions for data, docs and includes.
    actions += _install_actions(
        ctx,
        ctx.attr.docs,
        ctx.attr.doc_dest,
        strip_prefixes = ctx.attr.doc_strip_prefix,
        rename = rename,
    )
    actions += _install_actions(
        ctx,
        ctx.attr.data,
        ctx.attr.data_dest,
        strip_prefixes = ctx.attr.data_strip_prefix,
        rename = rename,
    )
    actions += _install_actions(
        ctx,
        ctx.attr.hdrs,
        ctx.attr.hdr_dest,
        strip_prefixes = ctx.attr.hdr_strip_prefix,
        rename = rename,
    )

    for t in ctx.attr.targets:
        # TODO(jwnimmer-tri): Raise an error if a target has testonly=1.
        if CcInfo in t:
            actions += _install_cc_actions(ctx, t)
        elif JavaInfo in t:
            actions += _install_java_actions(ctx, t)
        elif PyInfo in t:
            actions += _install_py_actions(ctx, t)
        elif hasattr(t, "files_to_run") and t.files_to_run.executable:
            # Executable scripts copied from source directory.
            actions += _install_runtime_actions(ctx, t)
        else:
            fail("Don't know how to install {}".format(t.label))

    # Generate install test actions.
    installed_tests += _install_test_actions(ctx)

    # Generate code for install actions.
    script_actions = []
    installed_files = {}
    for a in actions:
        src = None
        if hasattr(a, "src"):
            src = a.src
        if a.dst not in installed_files:
            if src:
                script_actions.append(_install_code(a))
            else:
                script_actions.append(_java_launcher_code(a))
            installed_files[a.dst] = src
        elif src != installed_files[a.dst]:
            fail("Install conflict detected:\n" +
                 "\n  src1 = " + repr(installed_files[a.dst]) +
                 "\n  src2 = " + repr(src) +
                 "\n  dst = " + repr(a.dst))

    # Generate installer manifest.
    actions_file = ctx.actions.declare_file(ctx.attr.name + "_actions")
    ctx.actions.write(
        output = actions_file,
        content = "\n".join(script_actions) + "\n",
    )

    # Generate install script.
    installer_binary = ctx.attr._installer[DefaultInfo]
    ctx.actions.write(
        output = ctx.outputs.executable,
        content = "\n".join([
            "#!/bin/bash",
            "{} --actions '{}' \"$@\"".format(
                installer_binary.files.to_list()[0].short_path,
                actions_file.short_path,
            ),
        ]),
        is_executable = True,
    )

    script_tests = []

    # Generate list containing all commands to run to test.
    for i in installed_tests:
        script_tests.append(i.cmd)

    # Generate test installation script.
    if ctx.attr.install_tests_script and not script_tests:
        fail("`install_tests_script` is not empty but no `script_tests` were provided.")  # noqa
    if ctx.attr.install_tests_script:
        ctx.actions.write(
            output = ctx.outputs.install_tests_script,
            content = "\n".join(script_tests),
            is_executable = False,
        )

    # Return actions.
    installer_runfiles = installer_binary.default_runfiles
    action_runfiles = ctx.runfiles(files = (
        [a.src for a in actions if not hasattr(a, "main_class")] +
        [i.src for i in installed_tests] +
        [actions_file]
    ))
    return [
        InstallInfo(
            install_actions = actions,
            rename = rename,
            installed_files = installed_files,
        ),
        InstalledTestInfo(tests = installed_tests),
        DefaultInfo(runfiles = installer_runfiles.merge(action_runfiles)),
    ]

# TODO(mwoehlke-kitware) default guess_data to PACKAGE when we have better
# default destinations.
_install_rule = rule(
    # Update buildifier-tables.json when this changes.
    attrs = {
        "deps": attr.label_list(providers = [InstallInfo]),
        "docs": attr.label_list(allow_files = True),
        "doc_dest": attr.string(default = "share/doc/@WORKSPACE@"),
        "doc_strip_prefix": attr.string_list(),
        "data": attr.label_list(allow_files = True),
        "data_dest": attr.string(default = "share/@WORKSPACE@"),
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
        "py_dest": attr.string(
            default = "lib/python@PYTHON_VERSION@/site-packages",
        ),
        "py_strip_prefix": attr.string_list(),
        "rename": attr.string_dict(),
        "install_tests": attr.label_list(
            default = [],
            allow_files = True,
        ),
        "workspace": attr.string(),
        "allowed_externals": attr.label_list(allow_files = True),
        "_installer": attr.label(
            executable = True,
            cfg = "host",
            default = Label("//tools/install:installer"),
        ),
        "install_script": attr.output(),
        "install_tests_script": attr.output(),
    },
    executable = True,
    implementation = _install_impl,
    toolchains = [
        # Used to discern the major.minor site-packages path to install into.
        _PY_CC_TOOLCHAIN_TYPE,
    ],
)

def install(tags = [], **kwargs):
    # (The documentation for this function is immediately below.)
    _install_rule(
        tags = tags + ["install"],
        **kwargs
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

Destination paths may include the following placeholders:

* ``@WORKSPACE@``, replaced with ``workspace`` (if specified) or the name of
  the workspace which invokes ``install``.
* ``@PYTHON_VERSION@``, replaced with the Python major.minor version.

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
    * ``ALLOWED_EXTERNALS``: (For guess_hdrs only) for each target, install
      source files which are used by the target and owned by a target in the
      at least one of the `allowed_externals = ...` list of labels.
    * ``EVERYTHING``: Install all headers/resources used by the target.

    The headers and resource files considered are *all* headers or resource
    files transitively used by the target. Any option other than ``NONE`` is
    also likely to install private headers, and may install resource files used
    by other targets. In either case, this may result in the same file being
    considered for installation more than once.

    Note also that, because Bazel includes *all* run-time dependencies —
    including e.g. shared libraries — in a target's ``runfiles``, only *source*
    artifacts are considered when guessing resource files.

    Java binary launchers are created at install time. The install script is
    configured to generate them, but no file other than the install script is
    created at build time. Java binary launchers rely on a target containing
    a ``MainClassInfo`` provider that contains all the required information to
    generate the launcher. Do not forget to provide as dependencies the install
    targets that rename files. This will be necessary to use the appropriate
    jar file name when creating the java launcher.

    MainClassInfo(
            main_class = Name of main class to run ("name.class.main")
            classpath = List contained in
                ctx.attr.target[JavaInfo].compilation_info.runtime_classpath
            filename = Java launcher file name
        )

    A file containing all the commands to test executables after installation
    is created if `install_tests_script` is set. The list of commands to run
    is given by `install_tests`. The generated file location can be passed to
    `install_test()` as an `args`.

Args:
    deps: List of other install rules that this rule should include.
    docs: List of documentation files to install.
    doc_dest: Destination for documentation files
        (default = "share/doc/@WORKSPACE@").
    doc_strip_prefix: List of prefixes to remove from documentation paths.
    guess_data: See note.
    guess_data_exclude: List of resources found by ``guess_data`` to exclude
        from installation.
    data: List of (platform-independent) resource files to install.
    data_dest: Destination for resource files (default = "share/@WORKSPACE@").
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
        (default = "lib/python{MAJOR}.{MINOR}/site-packages").
    py_strip_prefix: List of prefixes to remove from Python paths.
    rename: Mapping of install paths to alternate file names, used to rename
      files upon installation.
    install_tests: List of scripts that are designed to test the install
        tree. These scripts will not be installed.
    install_tests_script: Name of the generated file that contains the commands
        run to test the install tree. This only needs to be specified for the
        main `install()` call, and the same name should be passed to
        `install_test()` as `"$(location :" + install_tests_script + ")"`.
    workspace: Workspace name to use in default paths (overrides built-in
        guess).
    allowed_externals: List of external packages whose files may be installed.
"""

#------------------------------------------------------------------------------
# Generate information to install files to specified destination.
def _install_files_impl(ctx):
    # Get path components.
    dest = ctx.attr.dest
    strip_prefix = ctx.attr.strip_prefix

    # Generate actions.
    actions = _install_actions(
        ctx,
        ctx.attr.files,
        dest,
        strip_prefix,
        rename = ctx.attr.rename,
    )

    # Return computed actions.
    return [InstallInfo(install_actions = actions, rename = ctx.attr.rename)]

_install_files_rule = rule(
    # Update buildifier-tables.json when this changes.
    attrs = {
        "dest": attr.string(mandatory = True),
        "files": attr.label_list(allow_files = True),
        "rename": attr.string_dict(),
        "strip_prefix": attr.string_list(),
        "workspace": attr.string(),
        "allowed_externals": attr.label_list(allow_files = True),
    },
    implementation = _install_files_impl,
)

def install_files(tags = [], **kwargs):
    # (The documentation for this function is immediately below.)
    _install_files_rule(
        tags = tags + ["install"],
        **kwargs
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

Destination paths may include the placeholder ``@WORKSPACE``, which is replaced
with ``workspace`` (if specified) or the name of the workspace which invokes
``install``.

``install_files`` has the same caveats regarding external files as
:func:`install`.

Args:
    dest: Destination for files.
    files: List of files to install.
    strip_prefix: List of prefixes to remove from input paths.
    rename: Mapping of install paths to alternate file names, used to rename
      files upon installation.
    workspace: Workspace name to use in default paths (overrides built-in
        guess).
    allowed_externals: List of external packages whose files may be installed.
"""

#END rules
#==============================================================================
#BEGIN macros

#------------------------------------------------------------------------------
def install_cmake_config(
        package,
        versioned = True,
        name = "install_cmake_config",
        visibility = ["//visibility:private"]):
    """Generate installation information for CMake package configuration and
    package version files. The rule name is always ``:install_cmake_config``.

    Args:
        package (:obj:`str`): CMake package name.
        versioned (:obj:`bool`): True if a version file should be installed.
    """
    package_lower = package.lower()

    cmake_config_dest = "lib/cmake/{}".format(package_lower)
    cmake_config_files = ["{}-config.cmake".format(package_lower)]

    if versioned:
        cmake_config_files += ["{}-config-version.cmake".format(package_lower)]

    install_files(
        name = name,
        dest = cmake_config_dest,
        files = cmake_config_files,
        visibility = visibility,
    )

#------------------------------------------------------------------------------
def install_test(
        name,
        **kwargs):
    """A wrapper to test installed drake executables.

    !!!Important: This command should be called only once, when the main
    installation step occurs!!!

    This wrapper uses `//tools/install:install_test.py` as its main script. It
    expects to receive one argument which is the location of a file containing
    the list of command to run in the test. The current limitation requires
    each command to contain only one command per line. The file containing the
    list of command is typically `install_tests_script` outputted by the
    `install()` rule.
    """
    if native.package_name():
        fail("This command should be called only once, " +
             "when the main installation step occurs.")

    src = "//tools/install:install_test.py"

    # We can't use drake_py_unittest here, because the srcs path is atypical.
    drake_py_test(
        name = name,
        # This is an integration test with significant I/O that requires an
        # "eternal" timeout so that debug builds are successful.  Therefore,
        # the test size is increased to "medium", and the timeout to "eternal".
        # TODO(jamiesnape): Try to shorten the duration of this test.
        size = "medium",
        srcs = [src],
        timeout = "eternal",
        deps = [
            "//tools/install:install_test_helper",
            "//tools/install:otool",
        ],
        # Several install tests need some network access.
        allow_network = ["meshcat", "package_map"],
        # The commands in our "list of commands" use unittest themselves, so we
        # do the same for our own test rig.  That means that both our rig and
        # the "list of commands" python programs must have a __main__ clause
        # (i.e., they must all be binaries, not libraries).
        allow_import_unittest = True,
        **kwargs
    )

#END macros
