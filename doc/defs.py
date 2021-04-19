"""Common library to provide a reusable main() routine for all of our
documentation generation tools.
"""

import argparse
import functools
from http.server import SimpleHTTPRequestHandler
import os.path
from os.path import join
from socketserver import ThreadingTCPServer
import shlex
import subprocess
from subprocess import PIPE, STDOUT
import tempfile

from bazel_tools.tools.python.runfiles import runfiles

# This global variable can be toggled by our main() function.
_verbose = False


def verbose():
    """Returns True iff doc builds should produce detailed console output."""
    return _verbose


def symlink_input(filegroup_resource_path, temp_dir, strip_prefix=None):
    """Symlinks a rule's input data into a temporary directory.

    This is useful both to create a hermetic set of inputs to pass to a
    documentation builder, or also in case we need to adjust the input data
    before passing it along.

    Args:
        filegroup_resource_path: Names a file created by enumerate_filegroup
          (in defs.bzl) which contains resource paths.
        temp_dir: Destination directory, which must already exist.
        strip_prefix: Optional; a list[str] of candidate strings to remove
          from the resource path when linking into temp_dir.  The first match
          wins, and it is valid for no prefixes to match.
    """
    assert os.path.isdir(temp_dir)
    manifest = runfiles.Create()
    with open(manifest.Rlocation(filegroup_resource_path)) as f:
        input_filenames = f.read().splitlines()
    for name in input_filenames:
        orig_name = manifest.Rlocation(name)
        assert os.path.exists(orig_name), name
        dest_name = name
        for prefix in (strip_prefix or []):
            if dest_name.startswith(prefix):
                dest_name = dest_name[len(prefix):]
                break
        temp_name = join(temp_dir, dest_name)
        os.makedirs(os.path.dirname(temp_name), exist_ok=True)
        os.symlink(orig_name, temp_name)


def check_call(args, *, cwd=None):
    """Runs a subprocess command, raising an exception iff the process fails.

    Obeys the command-line verbosity flag for console output:
    - when in non-verbose mode, shows output only in case of an error;
    - when in verbose mode, shows the command-line and live output.

    Args:
        args: Passed to subprocess.run(args=...).
    """
    echo = "+ " + " ".join([shlex.quote(x) for x in args])
    if verbose():
        print(echo, flush=True)
        proc = subprocess.run(args, cwd=cwd, stderr=STDOUT)
    else:
        proc = subprocess.run(args, cwd=cwd, stderr=STDOUT, stdout=PIPE,
                              encoding='utf-8')
        if proc.returncode != 0:
            print(echo, flush=True)
            print(proc.stdout, end='', flush=True)
    proc.check_returncode()


def _call_build(*, build, out_dir):
    """Calls build() into out_dir, while also supplying a temp_dir."""
    with tempfile.TemporaryDirectory(
            dir=os.environ.get("TEST_TMPDIR"),
            prefix="doc_builder_temp_") as temp_dir:
        return build(out_dir=out_dir, temp_dir=temp_dir)


class _HttpHandler(SimpleHTTPRequestHandler):
    """An HTTP handler without logging."""

    def log_message(*_):
        pass

    def log_request(*_):
        pass


def _do_preview(*, build, subdir, port):
    """Implements the "serve" (http) mode of main().

    Args:
        build: Same as per main().
        subdir: Same as per main().
        port: Local port number to serve on, per the command line.
    """
    print("Generating documentation preview ...")
    with tempfile.TemporaryDirectory(prefix="doc_builder_preview_") as scratch:
        if subdir:
            out_dir = join(scratch, subdir)
            os.mkdir(out_dir)
        else:
            out_dir = scratch
        pages = _call_build(build=build, out_dir=out_dir)
        assert len(pages) > 0
        os.chdir(scratch)
        print(f"The files have temporarily been generated into {scratch}")
        print()
        print("Serving at the following URLs for local preview:")
        print()
        for page in pages:
            print(f"  http://127.0.0.1:{port}/{join(subdir, page)}")
        print()
        print("Use Ctrl-C to exit.")
        ThreadingTCPServer.allow_reuse_address = True
        server = ThreadingTCPServer(("127.0.0.1", port), _HttpHandler)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print()
            return


def _do_generate(*, build, out_dir, on_error):
    """Implements the "generate" (file output) mode of main().
    Args:
        build: Same as per main().
        out_dir: Directory to generate into, per the command line.
        on_error: Callback function to report problems with out_dir.
    """
    if out_dir == "<test>":
        out_dir = join(os.environ["TEST_TMPDIR"], "_builder_out")
    if not os.path.isabs(out_dir):
        on_error(f"--out_dir={out_dir} is not an absolute path")
    if os.path.exists(out_dir):
        if len(os.listdir(out_dir)) > 0:
            on_error(f"--out_dir={out_dir} is not empty")
    else:
        if verbose():
            print(f"+ mkdir -p {out_dir}", flush=True)
        os.makedirs(out_dir)
    print("Generating HTML ...")
    pages = _call_build(build=build, out_dir=out_dir)
    assert len(pages) > 0
    print("... done")


def main(*, build, subdir, description, supports_modules=False,
         supports_quick=False):
    """Reusable main() function for documentation binaries; processes
    command-line arguments and generates documentation.

    Args:
      build: Callback function to compile the documentation.
      subdir: A subdirectory to use when offering preview mode on a local web
        server; this does NOT affect the --out_dir path.
      description: Main help str for argparse; typically the caller's __doc__.
      supports_modules: Whether build() has a modules=list[str] argument.
      supports_quick: Whether build() has a quick=bool argument.
    """
    parser = argparse.ArgumentParser(description=description)
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--serve", action='store_true',
        help="Serve the documentation on the given PORT for easy preview.")
    group.add_argument(
        "--out_dir", type=str, metavar="DIR",
        help="Generate the documentation to the given output directory."
        " The DIR must be an absolute path."
        " If DIR already exists, then it must be empty."
        " (For regression testing, the DIR can be the magic value <test>,"
        " in which case a $TEST_TMPDIR subdir will be used.)")
    parser.add_argument(
        "--port", type=int, metavar="PORT", default=8000,
        help="Use a non-default PORT when serving for preview.")
    parser.add_argument(
        "--verbose", action="store_true",
        help="Echo detailed commands, progress, etc. to the console")
    if supports_modules:
        parser.add_argument(
            "module", nargs="*",
            help="Limit the generated documentation to only these modules and "
            "their children.  When none are provided, all will be generated. "
            "For example, specify drake.math or drake/math for the C++ "
            "module, or pydrake.math or pydrake/math for the Python module.")
    if supports_quick:
        parser.add_argument(
            "--quick", action="store_true", default=False,
            help="Omit from the output items that are slow to generate. "
            "This yields a faster preview, but the output will be incomplete.")
    args = parser.parse_args()
    if args.verbose:
        global _verbose
        _verbose = True
    curried_build = build
    if supports_modules:
        canonicalized_modules = [
            x.replace('/', '.')
            for x in args.module
        ]
        curried_build = functools.partial(
            curried_build, modules=canonicalized_modules)
    if supports_quick:
        curried_build = functools.partial(
            curried_build, quick=args.quick)
    if args.out_dir is None:
        assert args.serve
        _do_preview(build=curried_build, subdir=subdir, port=args.port)
    else:
        _do_generate(build=curried_build, out_dir=args.out_dir,
                     on_error=parser.error)


if __name__ == '__main__':
    main()
