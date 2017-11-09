# -*- mode: python -*-
# vi: set ft=python :

"""A collection of OS-related utilities intended for use in repository rules,
i.e., rules used by WORKSPACE files, not BUILD files.
"""

def exec_using_which(repository_ctx, command):
    """Run the given command (a list), using which to locate the executable named
    by the zeroth index of `command`.

    Return struct with attributes:
    - error (None when success, or else str message)
    - stdout (str command output, possibly empty)
    """

    # Find the executable.
    fullpath = repository_ctx.which(command[0])
    if fullpath == None:
        return struct(
            stdout = "",
            error = "could not find which '%s'" % command[0])

    # Run the executable.
    result = repository_ctx.execute([fullpath] + command[1:])
    if result.return_code != 0:
        error = "error %d running %r (command %r, stdout %r, stderr %r)" % (
            result.return_code,
            command[0],
            command,
            result.stdout,
            result.stderr)
        return struct(stdout = result.stdout, error = error)

    # Success.
    return struct(stdout = result.stdout, error = None)

def _make_result(error = None,
                 ubuntu_release = None,
                 macos_release = None):
    """Return a fully-populated struct result for determine_os, below."""
    return struct(
        error = error,
        is_macos = (macos_release != None),
        is_ubuntu = (ubuntu_release != None),
        ubuntu_release = ubuntu_release,
        macos_release = macos_release)

def _determine_linux(repository_ctx):
    """Handle determine_os on Linux."""

    # Shared error message text across different failure cases.
    error_prologue = "could not determine Linux distribution: "

    # Run sed to determine Linux NAME and VERSION_ID.
    sed = exec_using_which(repository_ctx, [
        "sed",
        "-n",
        "/^\(NAME\|VERSION_ID\)=/{s/[^=]*=//;s/\"//g;p}",
        "/etc/os-release"])
    if sed.error != None:
        return _make_result(error = error_prologue + sed.error)

    # Compute an identifying string, in the form of "$NAME $VERSION_ID".
    lines = [line.strip() for line in sed.stdout.strip().split("\n")]
    distro = " ".join([x for x in lines if len(x) > 0])

    # Match supported Ubuntu release(s).
    for ubuntu_release in ["16.04"]:
        if distro == "Ubuntu " + ubuntu_release:
            return _make_result(ubuntu_release = ubuntu_release)

    # Nothing matched.
    return _make_result(
        error = error_prologue + "unsupported distribution '%s'" % distro)

def _determine_macos(repository_ctx):
    """Handle determine_os on macOS."""

    # Shared error message text across different failure cases.
    error_prologue = "could not determine macOS version: "

    # Run sw_vers to determine macOS version.
    sw_vers = exec_using_which(repository_ctx, [
        "sw_vers",
        "-productVersion"])
    if sw_vers.error != None:
        return _make_result(error = error_prologue + sw_vers.error)

    major_minor_versions = sw_vers.stdout.strip().split(".")[:2]
    macos_release = ".".join(major_minor_versions)

    # Match supported macOS release(s).
    if macos_release in ["10.11", "10.12", "10.13"]:
        return _make_result(macos_release = macos_release)

    # Nothing matched.
    return _make_result(
        error = error_prologue + "unsupported macOS '%s'" % macos_release)

def determine_os(repository_ctx):
    """
    A repository_rule helper function that determines which of the supported OS
    versions we are targeting.

    Argument:
        repository_ctx: The context passed to the repository_rule calling this.

    Result:
        a struct, with attributes:
        - error: str iff any error occurred, else None
        - is_macos: True iff on a supported macOS release, else False
        - macos_release: str like "10.13" iff on a supported macOS, else None
        - is_ubuntu: True iff on a supported Ubuntu version, else False
        - ubuntu_release: str like "16.04" iff on a supported ubuntu, else None
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
        mapping: dict(str, list(str)) where the keys match the OS, and the list
            of values are of the form name=actual as in alias(name, actual).

    The keys of mapping are searched in the following preferential order:
    - Exact release, via e.g., "Ubuntu 16.04" or "macOS 10.12"
    - Any release, via "Ubuntu default" or "macOS default"
    - Anything else, via "default"
    """

    os_result = determine_os(repository_ctx)
    if os_result.error != None:
        fail(os_result.error)

    # Find the best match in the mapping dict for our OS.
    keys = []
    if os_result.ubuntu_release:
        keys = [
            "Ubuntu " + os_result.ubuntu_release,
            "Ubuntu default",
            "default",
        ]
    elif os_result.macos_release:
        keys = [
            "macOS " + os_result.macos_release,
            "macOS default",
            "default",
        ]
    found_items = None
    for key in keys:
        if key in mapping:
            found_items = mapping[key]
            break
    if not found_items:
        fail("Unsupported os_result " + repr(os_result))

    # Emit the list of aliases.
    file_content = "package(default_visibility = ['//visibility:public'])\n"
    for item in found_items:
        name, actual = item.split("=")
        file_content += 'alias(name = "{}", actual = "{}")\n'.format(
            name, actual)
    repository_ctx.file("BUILD", content = file_content, executable = False)

def _os_specific_alias_impl(repository_ctx):
    os_specific_alias(repository_ctx, repository_ctx.attr.mapping)

os_specific_alias_repository = repository_rule(
    attrs = {
        "mapping": attr.string_list_dict(mandatory = True),
    },
    implementation = _os_specific_alias_impl,
)
