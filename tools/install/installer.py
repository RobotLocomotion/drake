"""
Installation script generated from a Bazel `install` target.
"""

# N.B. Please ensure any significant behavior is tested by `install_meta_test`.
# Example execution:
#   $ bazel test //tools/install:py/install_meta_test --test_output=streamed --nocache_test_results  # noqa

# N.B. This is designed to emulate CMake's install mechanism. Do not add
# unnecessary print statements.

import argparse
import collections
import filecmp
import functools
import itertools
import os
from pathlib import Path
import re
import shutil
import stat
from subprocess import check_call, check_output
import sys

from python import runfiles

from tools.install import otool

# Used for matching against libraries and extracting useful components.
# N.B. On linux, dynamic libraries may have their version number as a suffix
# (e.g. my_lib.so.x.y.z).
dylib_match = re.compile(r"(.*\.so)(\.\d+)*$|(.*\.dylib)$")


def _is_relative_link(filepath):
    """Find if a file is a relative link.

    Bazel paths are assumed to always be absolute. If path is not absolute,
    the file is a link we want to keep.

    If the given `filepath` is not a link, the function returns `None`. If the
    given `filepath` is a link, the result will depend if the link is absolute
    or relative. The function is called recursively. If the result is not a
    link, `None` is returned. If the link is relative, the relative link is
    returned.
    """
    if os.path.islink(filepath):
        link = os.readlink(filepath)
        if not os.path.isabs(link):
            return link
        else:
            return _is_relative_link(link)
    else:
        return None


def _needs_install(src, dst, prefix):
    # Get canonical destination.
    dst_full = os.path.join(prefix, dst)

    # Check if destination exists.
    if not os.path.exists(dst_full):
        # Destination doesn't exist -> installation needed.
        return True

    # Check if files are different.
    if filecmp.cmp(src, dst_full, shallow=False):
        # Files are the same -> no installation needed.
        return False

    # File needs to be installed.
    return True


@functools.cache
def _patchelf_path() -> Path:
    manifest = runfiles.Create()
    return Path(manifest.Rlocation("patchelf/patchelf"))


class Installer:
    def __init__(self):
        # Stored from command-line.
        self.prefix = None
        self.strip = True
        self.strip_tool = None
        self.install_name_tool = None
        self.list_only = False

        # Mapping used to (a) check for unique shared library names, and
        # (b) provide a mapping from library name to paths for RPATH fixes
        # (where (a) is essential).
        # Structure: Map[ basename (Str) => full_path ]
        self._libraries_installed = {}

        # These are libraries (but not Python shared libraries) that require
        # RPATH (and thus depend on `libraries_installed`).
        # Structure: List[ Tuple(basename, full_path) ]
        self._libraries_to_fix_rpath = []

        # These are binaries (or Python shared libraries) that require RPATH
        # fixes, but by definition are not depended upon by other components,
        # and thus need not be unique.
        # Structure: List[ Tuple(basename, full_path) ]
        self._binaries_to_fix_rpath = []

    def copy_or_link(self, src, dst):
        """Copy file if it is not a relative link or recreate the symlink in
        `dst`.

        Copy the input file to the destination if it is not a relative link. If
        the file is a relative link, create a similar link in the destination
        folder.
        """
        relative_link = _is_relative_link(src)
        if relative_link:
            if os.path.exists(dst) or os.path.islink(dst):
                os.unlink(dst)
            os.symlink(relative_link, dst)
        else:
            shutil.copy2(src, dst)

    def install(self, src, dst):
        # In list-only mode, just display the filename, don't do any real work.
        if self.list_only:
            print(dst)
            return

        # Ensure destination subdirectory exists, creating it if necessary.
        subdir = os.path.dirname(dst)
        subdir_full = os.path.join(self.prefix, subdir)
        os.makedirs(subdir_full, exist_ok=True)

        dst_full = os.path.join(self.prefix, dst)
        # Install file, if not up to date.
        if _needs_install(src, dst, self.prefix):
            print(f"-- Installing: {dst_full}")
            if os.path.exists(dst_full):
                os.remove(dst_full)
            self.copy_or_link(src, dst_full)
            needs_patching = True
        else:
            # TODO(mwoehlke-kitware): Stage RPATH changes so we don't need to
            # install libraries that haven't changed? (Currently, unless a
            # "patched" file is identical to the as-built version, we won't get
            # here, which means in practice any libraries that had RPATH
            # patching are always reinstalled.)
            print(f"-- Up-to-date: {dst_full}")
            needs_patching = False

        basename = os.path.basename(dst)
        if re.match(dylib_match, basename):
            # It is a library.
            if dst.startswith("lib/python") and not basename.startswith("lib"):
                # Assume this is a Python C extension.
                self._binaries_to_fix_rpath.append((basename, dst_full))
            else:
                # Check that dependency is only referenced once in the library
                # dictionary. If it is referenced multiple times, we do not
                # know which one to use, and fail fast.
                if basename in self._libraries_installed:
                    sys.stderr.write(
                        f"Multiple installation rules found for {basename}.\n"
                    )
                    sys.exit(1)
                self._libraries_installed[basename] = dst_full
                if needs_patching:
                    self._libraries_to_fix_rpath.append((basename, dst_full))

    def fix_rpaths_and_strip(self):
        # Only fix files that are installed now.
        fix_items = itertools.chain(
            self._libraries_to_fix_rpath, self._binaries_to_fix_rpath
        )
        for basename, dst_full in fix_items:
            if os.path.islink(dst_full):
                # Skip files that are links. However, they need to be in the
                # dictionary to fixup other library and executable paths.
                continue
            # Enable write permissions to allow modification.
            os.chmod(
                dst_full,
                stat.S_IRUSR
                | stat.S_IWUSR
                | stat.S_IXUSR
                | stat.S_IRGRP
                | stat.S_IXGRP
                | stat.S_IROTH
                | stat.S_IXOTH,
            )
            if sys.platform == "darwin":
                # From the manual for BSD `strip`: for dynamic shared
                # libraries, the maximum level of stripping is usually -x (to
                # remove all non-global symbols).
                if self.strip:
                    check_call([self.strip_tool, "-x", dst_full])
                self._macos_fix_rpaths(basename, dst_full)
            else:
                # Strip before running `patchelf`. Trying to strip after
                # patching the files is likely going to create the following
                # error:
                #   'Not enough room for program headers, try linking with -N'
                if self.strip:
                    check_call([self.strip_tool, dst_full])
                self._linux_fix_rpaths(dst_full)

    def _macos_fix_rpaths(self, basename, dst_full):
        # Update file (library, executable) ID (remove relative path).
        check_call(
            [self.install_name_tool, "-id", "@rpath/" + basename, dst_full]
        )
        # Check if file dependencies are specified with relative paths.
        for dep in otool.linked_libraries(dst_full):
            # Look for the absolute path in the dictionary of fixup files to
            # find library paths.
            if dep.basename not in self._libraries_installed:
                continue
            lib_dirname = os.path.dirname(dst_full)
            diff_path = os.path.relpath(
                self._libraries_installed[dep.basename], lib_dirname
            )
            check_call(
                [
                    self.install_name_tool,
                    "-change",
                    dep.path,
                    os.path.join("@loader_path", diff_path),
                    dst_full,
                ]
            )
        # Remove RPATH values that contain @loader_path. These are from the
        # build tree and are irrelevant in the install tree. RPATH is not
        # necessary as relative or absolute path to each library is already
        # known.
        for command in otool.load_commands(dst_full):
            if command["cmd"] != "LC_RPATH" or "path" not in command:
                continue

            path = command["path"]
            if path.startswith("@loader_path"):
                check_call(
                    [self.install_name_tool, "-delete_rpath", path, dst_full]
                )

    def _is_non_local_library(self, entry):
        return entry == "not found" or entry.startswith(self.prefix)

    def _linux_fix_rpaths(self, dst_full):
        # A conservative subset of the ld.so search path. These paths are added
        # to /etc/ld.so.conf by default or after the prerequisites install
        # script has been run. Query on a given system using `ldconfig -v`.
        ld_so_search_paths = [
            "/lib",
            "/lib/libblas",
            "/lib/liblapack",
            "/lib/x86_64-linux-gnu",
            "/lib32",
            "/libx32",
            "/usr/lib",
            "/usr/lib/x86_64-linux-gnu",
            "/usr/lib/x86_64-linux-gnu/libfakeroot",
            "/usr/lib/x86_64-linux-gnu/mesa-egl",
            "/usr/lib/x86_64-linux-gnu/mesa",
            "/usr/lib/x86_64-linux-gnu/pulseaudio",
            "/usr/lib32",
            "/usr/libx32",
            "/usr/local/lib",
        ]
        file_output = check_output(["ldd", dst_full]).decode("utf-8")
        rpath = []
        for line in file_output.splitlines():
            ldd_result = line.strip().split(" => ")
            if len(ldd_result) < 2:
                continue
            # Library in install prefix.
            if self._is_non_local_library(ldd_result[1]):
                re_result = re.match(dylib_match, ldd_result[0])
                # Look for the absolute path in the dictionary of libraries
                # using the library name without its possible version number.
                soname, version, _ = re_result.groups()
                if soname not in self._libraries_installed:
                    # Some third party libraries that are copied rather than
                    # compiled, such as mosek, are stored as keys in
                    # libraries_installed with the version
                    # (e.g., libname.so.1).
                    soname = f"{soname}{version}"
                    if soname not in self._libraries_installed:
                        continue
                lib_dirname = os.path.dirname(dst_full)
                diff_path = os.path.dirname(
                    os.path.relpath(
                        self._libraries_installed[soname], lib_dirname
                    )
                )
                rpath.append("$ORIGIN" + "/" + diff_path)
            # System library not in ld.so search path.
            else:
                # Remove (hexadecimal) address from output leaving (at most)
                # the path to the library.
                ldd_regex = r"(.*\.so(?:\.\d+)*) \(0x[0-9a-f]+\)$"
                re_result = re.match(ldd_regex, ldd_result[1])
                if re_result:
                    lib_dirname = os.path.dirname(
                        os.path.realpath(re_result.group(1))
                    )
                    if lib_dirname not in ld_so_search_paths:
                        rpath.append(lib_dirname + "/")

        # The above may have duplicated some items into the list.  Uniquify it
        # here, preserving order.  Note that we do not just use a set() above,
        # since order matters.
        rpath = collections.OrderedDict.fromkeys(rpath).keys()

        # Replace build tree RPATH with computed install tree RPATH. Build tree
        # RPATH are automatically removed by this call. RPATH will contain the
        # necessary absolute and relative paths to find the libraries that are
        # needed. RPATH will typically be set to `$ORIGIN` or
        # `$ORIGIN/../../..`, possibly concatenated with directories under
        # /opt.
        str_rpath = ":".join(x for x in rpath)
        check_output(
            [
                _patchelf_path(),
                "--force-rpath",  # We need to override LD_LIBRARY_PATH.
                "--set-rpath",
                str_rpath,
                dst_full,
            ]
        )

    def create_java_launcher(self, filename, classpath, jvm_flags, main_class):
        # In list-only mode, just display the filename, don't do any real work.
        if self.list_only:
            print(filename)
            return

        filename = os.path.join(self.prefix, filename)
        print(f"-- Generating: {filename}")

        # Make sure install directory exists.
        filepath = os.path.dirname(filename)
        if not os.path.exists(filepath):
            os.makedirs(filepath)

        # Converting classpath to string
        strclasspath = '"{}"'.format('" "'.join(classpath))

        # Write launcher.
        if os.path.exists(filename):
            os.chmod(filename, stat.S_IWUSR)
        with open(filename, "w") as launcher_file:
            launcher_file.write(f"""#!/bin/bash
# autogenerated - do not edit.
set -euo pipefail

# Seek our jar dependencies on Drake's install path.
# This file is installed to <prefix>/bin.
prefix=$(python3 -c 'import os;\
    print(os.path.realpath(os.path.join(\"'\"$0\"'\", os.pardir, os.pardir)))')
for jar_file in {strclasspath}; do
  if [ -f "$jar_file" ]; then
    export CLASSPATH="${{CLASSPATH:+$CLASSPATH:}}$jar_file"
  fi
done

# Seek our jar dependencies on the host system.
prefix=/usr
for jar_file in {strclasspath}; do
  if [ -f "$jar_file" ]; then
    export CLASSPATH="${{CLASSPATH:+$CLASSPATH:}}$jar_file"
  fi
done

java {jvm_flags} {main_class} "$@"
""")
        os.chmod(
            filename,
            stat.S_IRUSR
            | stat.S_IRGRP
            | stat.S_IROTH
            | stat.S_IXUSR
            | stat.S_IXGRP
            | stat.S_IXOTH,
        )


def main(args):
    installer = Installer()

    # Set up options.
    parser = argparse.ArgumentParser()
    parser.add_argument("prefix", type=str, help="Install prefix")
    parser.add_argument(
        "--actions",
        type=str,
        required=True,
        help="file path to installer actions",
    )
    parser.add_argument(
        "--install_name_tool",
        type=str,
        default="install_name_tool",
        help="install_name_tool program",
    )
    parser.add_argument(
        "--list",
        dest="list_only",
        action="store_true",
        default=False,
        help="print the list of installed files; do not install anything",
    )
    parser.add_argument(
        "--no_strip",
        dest="strip",
        action="store_false",
        default=True,
        help="do not strip symbols (for debugging)",
    )
    parser.add_argument(
        "--strip_tool", type=str, default="strip", help="strip program"
    )
    parser.add_argument(
        "--pre_clean",
        action="store_true",
        default=False,
        help="ensure clean install by removing `prefix` dir if it exists "
        "before installing",
    )
    args = parser.parse_args(args)

    # Get install prefix.
    installer.prefix = args.prefix
    installer.list_only = args.list_only
    # Check if we want to avoid stripping symbols.
    installer.strip = args.strip
    installer.strip_tool = args.strip_tool
    installer.install_name_tool = args.install_name_tool
    pre_clean = args.pre_clean

    # Transform install prefix if DESTDIR is set.
    # https://www.gnu.org/prep/standards/html_node/DESTDIR.html
    destdir = os.environ.get("DESTDIR")
    if destdir:
        installer.prefix = destdir + installer.prefix

    # Because Bazel executes us in a strange working directory and not the
    # working directory of the user's shell, enforce that the install
    # location is an absolute path so that the user is not surprised.
    if not os.path.isabs(installer.prefix):
        parser.error(
            "Install prefix must be an absolute path"
            f" (got '{installer.prefix}')\n"
        )

    if pre_clean:
        if os.path.isdir(installer.prefix):
            print(f"Remove previous directory: {installer.prefix}")
            shutil.rmtree(installer.prefix)

    if installer.strip:
        # Match the output of the CMake install/strip target
        # (https://git.io/fpdzK).
        print("Installing the project stripped...", sep="")
    else:
        # Match the output of the CMake install target (https://git.io/fpdzo).
        print("Install the project...", sep="")

    # Execute the install actions.
    # TODO(jwnimmer-tri) Executing arbitrary Python code from the actions file
    # is an absurd implementation choice that we've inherited from the original
    # installer scripts.  We should rework the install.bzl <=> installer.py
    # specification format to use something other than open-ended Python code.
    for action in open(args.actions, "r", encoding="utf-8"):
        exec(f"installer.{action}")

    # Libraries paths may need to be updated in libraries and executables.
    installer.fix_rpaths_and_strip()


if __name__ == "__main__":
    main(sys.argv[1:])
