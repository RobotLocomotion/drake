# -*- mode: python -*-
# vi: set ft=python :

"""A collection of OS-related utilities intended for use in repository rules,
i.e., rules used by WORKSPACE files, not BUILD files.

To opt-in to the "manylinux" build variant, set the environment variable
`DRAKE_OS=manylinux` before running the build.  The most precise way to do
this is to add a `user.bazelrc` file to the root of the Drake source tree
with the following content:

  common --repo_env=DRAKE_OS=manylinux

Alternatively, you may pass `--repo_env=DRAKE_OS=manylinux` on the bazel
command line.
"""

load("@drake//tools/workspace:execute.bzl", "which")

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
        ubuntu_release = None,
        macos_release = None,
        is_manylinux = False,
        homebrew_prefix = None):
    """Return a fully-populated struct result for determine_os, below."""
    if ubuntu_release != None:
        distribution = "ubuntu"
    elif macos_release != None:
        distribution = "macos"
    elif is_manylinux:
        distribution = "manylinux"
    else:
        distribution = None
    return struct(
        error = error,
        distribution = distribution,
        is_macos = (macos_release != None),
        is_ubuntu = (ubuntu_release != None and not is_manylinux),
        is_manylinux = is_manylinux,
        ubuntu_release = ubuntu_release,
        macos_release = macos_release,
        homebrew_prefix = homebrew_prefix,
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

    if distro == "Ubuntu":
        lsb = exec_using_which(repository_ctx, ["lsb_release", "-sr"])
        if lsb.error != None:
            return _make_result(error = error_prologue + lsb.error)
        ubuntu_release = lsb.stdout.strip()

        # Match supported Ubuntu release(s). These should match those listed in
        # both doc/_pages/from_source.md and the root CMakeLists.txt.
        if ubuntu_release in ["20.04"]:
            return _make_result(
                ubuntu_release = ubuntu_release,
                is_manylinux = is_manylinux,
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

    # Run sw_vers to determine macOS version.
    sw_vers = exec_using_which(repository_ctx, [
        "sw_vers",
        "-productVersion",
    ])
    if sw_vers.error != None:
        return _make_result(error = error_prologue + sw_vers.error)

    major_minor_versions = sw_vers.stdout.strip().split(".")[:2]
    if int(major_minor_versions[0]) < 11:
        macos_release = ".".join(major_minor_versions)
    else:
        macos_release = major_minor_versions[0]

    # Check which arch we should be using.
    arch_result = exec_using_which(repository_ctx, ["/usr/bin/arch"])
    if arch_result.stdout.strip() == "arm64":
        homebrew_prefix = "/opt/homebrew"
    else:
        homebrew_prefix = "/usr/local"

    # Match supported macOS release(s).
    if macos_release in ["11", "12"]:
        return _make_result(
            macos_release = macos_release,
            homebrew_prefix = homebrew_prefix,
        )

    # Nothing matched.
    return _make_result(
        error = error_prologue + "unsupported macOS '%s'" % macos_release,
    )

def determine_os(repository_ctx):
    """
    A repository_rule helper function that determines which of the supported OS
    versions we are targeting.

    Note that even if the operating system hosting the build is Ubuntu, the
    target OS might be "manylinux", which means that we only use the most basic
    host packages from Ubuntu (libc, libstdc++, etc.).  In that case, the
    value of is_ubuntu will be False.

    Argument:
        repository_ctx: The context passed to the repository_rule calling this.

    Result:
        a struct, with attributes:
        - error: str iff any error occurred, else None
        - distribution: str either "ubuntu" or "macos" or "manylinux" iff no
                        error occurred, else None
        - is_macos: True iff on a supported macOS release, else False
        - macos_release: str like "11" or "12" iff on a supported macOS,
                         else None
        - homebrew_prefix: str "/usr/local" or "/opt/homebrew" iff is_macos,
                           else None.
        - is_ubuntu: True iff on a supported Ubuntu version, else False
        - ubuntu_release: str like "20.04" iff on a supported ubuntu, else None
        - is_manylinux: True iff this build will be packaged into a Python
                        wheel that confirms to a "manylinux" standard such as
                        manylinux_2_27; see https://github.com/pypa/manylinux.
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
    - Exact release, via e.g., "Ubuntu 20.04" or "macOS 11"
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
    elif os_result.is_manylinux:
        keys = [
            "manylinux",
        ]
    found_items = None
    for key in keys:
        if key in mapping:
            found_items = mapping[key]
            break
    if not found_items:
        fail("Unsupported os_result " + repr(os_result))

    # Emit the list of aliases.
    file_content = """# -*- python -*-

# DO NOT EDIT: generated by os_specific_alias_repository()

package(default_visibility = ["//visibility:public"])
"""

    for item in found_items:
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

def _os_impl(repo_ctx):
    os_result = determine_os(repo_ctx)
    repo_ctx.file("BUILD.bazel", "")

    if os_result.error:
        fail(os_result.error)

    constants = """
DISTRIBUTION = {distribution}
UBUNTU_RELEASE = {ubuntu_release}
MACOS_RELEASE = {macos_release}
HOMEBREW_PREFIX = {homebrew_prefix}
    """.format(
        distribution = repr(os_result.distribution),
        ubuntu_release = repr(os_result.ubuntu_release),
        macos_release = repr(os_result.macos_release),
        homebrew_prefix = repr(os_result.homebrew_prefix),
    )
    repo_ctx.file("os.bzl", constants)

os_repository = repository_rule(
    implementation = _os_impl,
)

"""
Provides the fields `DISTRIBUTION`, `UBUNTU_RELEASE` and `MACOS_RELEASE` from
`determine_os`.
"""
