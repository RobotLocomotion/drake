"""A collection of OS-related utilities intended for use in repository rules,
i.e., rules used by WORKSPACE files, not BUILD files.

To opt-in to the "manylinux" or "macos_wheel" build variants, set the
environment variable (e.g.) `DRAKE_OS=manylinux` before running the build.  The
most precise way to do this is to add a `user.bazelrc` file to the root of the
Drake source tree with the following content:

  common --repo_env=DRAKE_OS=manylinux

Alternatively, you may pass `--repo_env=DRAKE_OS=manylinux` on the bazel
command line. (Replace "manylinux" with "macos_wheel" as appropriate.)
"""

load("//tools/workspace:execute.bzl", "which")

def exec_using_which(repository_ctx, command):
    """Run the given command (a list), using the which() function in
    execute.bzl to locate the executable named by the zeroth index of
    `command`.

    Return struct with attributes:
    - error (None when success, or else str message)
    - stdout (str command output, possibly empty)
    """

    # Find the executable.
    fullpath = which(repository_ctx, command[0])
    if fullpath == None:
        return struct(
            stdout = "",
            error = "could not find which '%s'" % command[0],
        )

    # Run the executable.
    result = repository_ctx.execute([fullpath] + command[1:])
    if result.return_code != 0:
        error = "error %d running %r (command %r, stdout %r, stderr %r)" % (
            result.return_code,
            command[0],
            command,
            result.stdout,
            result.stderr,
        )
        return struct(stdout = result.stdout, error = error)

    # Success.
    return struct(stdout = result.stdout, error = None)

def _make_result(
        error = None,
        macos_release = None,
        ubuntu_release = None,
        is_wheel = False,
        homebrew_prefix = None,
        macos_arch_result = None):
    """Return a fully-populated struct result for determine_os, below."""
    is_macos = (macos_release != None) and not is_wheel
    is_macos_wheel = (macos_release != None) and is_wheel
    is_ubuntu = (ubuntu_release != None) and not is_wheel
    is_manylinux = (ubuntu_release != None) and is_wheel
    if is_macos:
        target = "macos"
    elif is_macos_wheel:
        target = "macos_wheel"
    elif is_ubuntu:
        target = "ubuntu"
    elif is_manylinux:
        target = "manylinux"
    else:
        target = None
    return struct(
        error = error,
        target = target,
        is_macos = is_macos,
        is_macos_wheel = is_macos_wheel,
        is_ubuntu = is_ubuntu,
        is_manylinux = is_manylinux,
        ubuntu_release = ubuntu_release,
        macos_release = macos_release,
        homebrew_prefix = homebrew_prefix,
        macos_arch_result = macos_arch_result,
    )

def _determine_linux(repository_ctx):
    """Handle determine_os on Linux."""

    # Shared error message text across different failure cases.
    error_prologue = "could not determine Linux distribution: "

    # Allow the user to override the OS selection.
    drake_os = repository_ctx.os.environ.get("DRAKE_OS", "")
    is_manylinux = False
    if len(drake_os) > 0:
        if drake_os == "manylinux":
            is_manylinux = True
        else:
            return _make_result(error = "{}{} DRAKE_OS={}".format(
                error_prologue,
                "unknown value for environment variable",
                drake_os,
            ))

    # Get distro name.
    lsb = exec_using_which(repository_ctx, ["lsb_release", "-si"])
    if lsb.error != None:
        return _make_result(error = error_prologue + lsb.error)
    distro = lsb.stdout.strip()

    # Some Ubuntu derivatives reply with their specific name in `-i` mode but
    # declare Ubuntu compatibility in /etc. Check for that as a fallback.
    if distro != "Ubuntu":
        result = repository_ctx.execute([
            "/bin/sh",
            "-c",
            ". /etc/lsb-release; echo $DISTRIB_ID",
        ])
        if result.return_code == 0:
            maybe_distro = result.stdout.strip()
            if maybe_distro == "Ubuntu":
                distro = maybe_distro

    if distro == "Ubuntu":
        lsb = exec_using_which(repository_ctx, ["lsb_release", "-sr"])
        if lsb.error != None:
            return _make_result(error = error_prologue + lsb.error)
        ubuntu_release = lsb.stdout.strip()

        # Match supported Ubuntu release(s). These should match those listed in
        # both doc/_pages/from_source.md and the root CMakeLists.txt.
        if ubuntu_release in ["22.04", "24.04"]:
            return _make_result(
                ubuntu_release = ubuntu_release,
                is_wheel = is_manylinux,
            )

        # Nothing matched.
        return _make_result(
            error = (error_prologue +
                     "unsupported '%s' release '%s'" %
                     (distro, ubuntu_release)),
        )

    # Nothing matched.
    return _make_result(
        error = error_prologue + "unsupported distribution '%s'" % distro,
    )

def _determine_macos(repository_ctx):
    """Handle determine_os on macOS."""

    # Shared error message text across different failure cases.
    error_prologue = "could not determine macOS version: "

    # Allow the user to override the OS selection.
    drake_os = repository_ctx.os.environ.get("DRAKE_OS", "")
    is_macos_wheel = False
    if len(drake_os) > 0:
        if drake_os == "macos_wheel":
            is_macos_wheel = True
        else:
            return _make_result(error = "{}{} DRAKE_OS={}".format(
                error_prologue,
                "unknown value for environment variable",
                drake_os,
            ))

    # Run sw_vers to determine macOS version.
    sw_vers = exec_using_which(repository_ctx, [
        "sw_vers",
        "-productVersion",
    ])
    if sw_vers.error != None:
        return _make_result(error = error_prologue + sw_vers.error)

    # Match supported macOS release(s).
    (macos_release,) = sw_vers.stdout.strip().split(".")[:1]
    if macos_release not in ["12", "13", "14"]:
        print("WARNING: unsupported macOS '%s'" % macos_release)

    # Check which arch we should be using.
    arch_result = exec_using_which(repository_ctx, ["/usr/bin/arch"])
    macos_arch_result = arch_result.stdout.strip()
    if macos_arch_result == "arm64":
        homebrew_prefix = "/opt/homebrew"
    else:
        homebrew_prefix = "/usr/local"

    return _make_result(
        macos_release = macos_release,
        is_wheel = is_macos_wheel,
        homebrew_prefix = homebrew_prefix,
        macos_arch_result = macos_arch_result,
    )

def determine_os(repository_ctx):
    """
    DO NOT USE THIS IN NEW CODE. We are working to remove this from Drake.

    A repository_rule helper function that determines which of the supported
    build environments (OS versions or wheel environments) we should target.

    We support four options, which are mutually exclusive and collectively
    exhaustive: "macos" or "macos_wheel" or "ubuntu" or "manylinux".

    The "manylinux" target indicates this build will be packaged into a Python
    wheel that conforms to a "manylinux" standard such as manylinux_2_31; see
    https://github.com/pypa/manylinux. Currently we compile this in an Ubuntu
    container using only the most basic host packages from Ubuntu (libc,
    libstdc++, etc.). In this case, the value of is_ubuntu will be False, but
    ubuntu_release will still be provided.

    The "macos_wheel" target indicates this build will be packaged into a
    Python wheel.

    In case of an error, the "error" attribute of the struct will be set, and
    all of the other fields will be None or False.

    Argument:
        repository_ctx: The context passed to the repository_rule calling this.

    Result:
        a struct, with attributes:
        - error: str iff any error occurred, else None

        - target: str "macos" or "macos_wheel" or "ubuntu" or "manylinux"
        - is_macos: True iff targeting a macOS non-wheel build
        - is_macos_wheel: True iff targeting a macOS wheel build
        - is_ubuntu: True iff targeting an Ubuntu non-wheel build
        - is_manylinux: True iff targeting a Linux wheel build

        - ubuntu_release: str like "22.04" (set any time the build
            platform is Ubuntu, even for builds targeting "manylinux")
        - macos_release: str like "11" or "12" (set any time the build platform
            is macOS, even for builds targeting "macos_wheel")
        - homebrew_prefix: str "/usr/local" or "/opt/homebrew" (set any time
            the build platform is macOS, even for builds targeting
            "macos_wheel")
    """

    os_name = repository_ctx.os.name
    if os_name == "mac os x":
        return _determine_macos(repository_ctx)
    elif os_name == "linux":
        return _determine_linux(repository_ctx)
    else:
        return _make_result(error = "unknown or unsupported OS '%s'" % os_name)

def os_specific_alias(repository_ctx, mapping):
    """
    A repository_rule helper function that creates a BUILD file with alias()
    declarations based on which supported OS version we are targeting.

    Argument:
        repository_ctx: The context passed to the repository_rule calling this.
        mapping: dict(str, list(str)) where the keys match the OS (which must
            be either "linux" or "osx", and the list of values are of the form
            name=actual as in alias(name, actual).
    """

    key = repository_ctx.os.name
    if key == "mac os x":
        key = "osx"

    if key not in mapping:
        fail("Unsupported os.name " + key)
    items = mapping[key]

    # Emit the list of aliases.
    file_content = """\
# DO NOT EDIT: generated by os_specific_alias_repository()

package(default_visibility = ["//visibility:public"])
"""

    for item in items:
        name, actual = item.split("=")
        file_content += 'alias(name = "{}", actual = "{}")\n'.format(
            name,
            actual,
        )
    repository_ctx.file(
        "BUILD.bazel",
        content = file_content,
        executable = False,
    )

def _os_specific_alias_impl(repository_ctx):
    os_specific_alias(repository_ctx, repository_ctx.attr.mapping)

os_specific_alias_repository = repository_rule(
    attrs = {
        "mapping": attr.string_list_dict(mandatory = True),
    },
    implementation = _os_specific_alias_impl,
)
