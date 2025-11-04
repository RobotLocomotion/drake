"""
Pythonic wrappers for macOS `otool`.
"""

from collections import namedtuple
import os
import re
import subprocess

Library = namedtuple(
    "Library",
    [
        "basename",
        "path",
        "version_compat",
        "version_current",
    ],
)

# Known Load command keys that contain spaces; DO NOT MODIFY at runtime.
_load_command_keys = (
    "time stamp",
    "current version",
    "compatibility version",
)


def _join(proc, cmd="otool"):
    """
    Wait for process `proc` to terminate, and raise an exception if it did not
    exit successfully (i.e. gave a non-zero exit code).
    """
    retcode = proc.wait(timeout=30)
    if retcode:
        raise subprocess.CalledProcessError(retcode, cmd)


def _split_load_command(line):
    """
    Splits a load command line into a key, value pair. Handles known key names
    that contain spaces.
    """
    for key in _load_command_keys:
        if line.startswith(f"{key} "):
            return [key, line[len(key) :].lstrip()]

    return line.split(" ", 1)


def load_commands(path):
    """
    Obtains the load commands of a Mach-O binary. Returns a list of commands,
    where each command is a dictionary describing the command. The key 'cmd'
    is always present and describes the type of command. The command type will
    determine what other keys are present.

    For the most part, values (e.g. time stamps) are not translated. As a
    partial exception, strings of the form 'value (offset X)' are split, and
    the offset is stored as 'Y:offset', where 'Y' is the corresponding key
    name. As another exception, 'cmdsize' is translated to ``int``.
    """
    commands = []
    command = None

    proc = subprocess.Popen(
        ["otool", "-l", path],
        stdout=subprocess.PIPE,
        text=True,
    )

    for line in proc.stdout:
        # Output looks like::
        #
        #   Load command N
        #        cmd LC_FOO
        #    cmdsize 64
        #       key1 value1 (offset 16)
        #       key2 value2
        #  extended key value
        #  Section
        #       key1 value1
        #       key2 value2
        #
        # Key names may or may not be indented, and some key names contain
        # spaces. Most values are aligned to a particular (but varying) column,
        # but long key names may change this column.
        if line.startswith("Load command"):
            if command is not None and len(command):
                commands.append(command)

            command = {}

        elif line == "Section\n":
            if command is not None and len(command):
                commands.append(command)

            command = None

        elif command is not None:
            kv = _split_load_command(line.strip())
            if len(kv) == 2:
                m = re.match("^(.*) [(]offset ([0-9]+)[)]$", kv[1].strip())
                if m is None:
                    if kv[0] == "cmdsize":
                        command[kv[0]] = int(kv[1].strip())
                    else:
                        command[kv[0]] = kv[1].strip()
                else:
                    command[kv[0]] = m.group(1).strip()
                    command[f"{kv[0]}:offset"] = int(m.group(2))

    _join(proc)

    return commands


def linked_libraries(path):
    """
    Obtains the set of shared libraries referenced by a Mach-O binary. Returns
    a list of Library objects.
    """
    libs = []

    proc = subprocess.Popen(
        ["otool", "-L", path],
        stdout=subprocess.PIPE,
        text=True,
    )

    for line in proc.stdout:
        # Output looks like (note that actual indent uses '\t')::
        #
        #   /path/to/input.dylib
        #       @rpath/libfoo.0.dylib <version>
        #   /usr/lib/libbar.5.dylib <version>
        #   ...
        #
        # <version> looks like '(compatibility version 1.0.0, '
        # 'current version 5.0.0)'.
        m = re.match("^\t(.*)[(]([^)]+)[)]\\s*$", line)
        if m is not None:
            path = m.group(1).strip()

            m = re.match(
                "^compatibility version (.*), current version (.*)$",
                m.group(2).strip(),
            )
            if m is not None:
                compat = m.group(1)
                current = m.group(2)
            else:
                compat = None
                current = None

            libs.append(
                Library(
                    path=path,
                    basename=os.path.basename(path),
                    version_compat=compat,
                    version_current=current,
                )
            )

    _join(proc)

    return libs
