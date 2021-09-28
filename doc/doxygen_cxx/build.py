"""Command-line tool to generate Drake's C++ API reference.

For instructions, see https://drake.mit.edu/documentation_instructions.html.
"""

import argparse
from fnmatch import fnmatch
import os
from os.path import join, relpath
import sys

from bazel_tools.tools.python.runfiles import runfiles

from drake.doc.defs import check_call, main, symlink_input, verbose


def _symlink_headers(*, drake_workspace, temp_dir, modules):
    """Prepare the input and output folders. We will copy the requested input
    file(s) into a temporary scratch directory, so that Doxygen doesn't scan
    the drake_workspace directly (which is extremely slow).
    """
    # Locate the default top-level modules.
    unwanted_top_level_dirs = [
        ".*",           # There is no C++ code here.
        "bazel-*",      # Ignore Bazel build artifacts.
        "build",        # Ignore CMake build artifacts.
        "cmake",        # There is no C++ code here.
        "debian",       # Ignore Debian build artifacts.
        "doc",          # There is no C++ code here.
        "gen",          # Ignore setup artifacts.
        "setup",        # There is no C++ code here.
        "third_party",  # Only document first-party Drake code.
        "tools",        # There is no C++ code here.
        "tutorials",    # There is no C++ code here.
    ]
    default_modules = [
        f"drake.{x}" for x in os.listdir(drake_workspace)
        if os.path.isdir(join(drake_workspace, x))
        and not any([
            fnmatch(x, unwanted)
            for unwanted in unwanted_top_level_dirs
        ])
    ]

    # Iterate modules one by one.
    for module in (modules or default_modules):
        if verbose():
            print(f"Symlinking {module} ...")
        prefix = "drake."
        if not module.startswith(prefix):
            print("error: Doxygen modules must start with 'drake',"
                  f" not {module}")
            sys.exit(1)
        module_as_subdir = module[len(prefix):].replace('.', '/')
        module_workspace = join(drake_workspace, module_as_subdir)
        if not os.path.isdir(module_workspace):
            print(f"error: Unknown module {module}")
            sys.exit(1)
        for dirpath, dirs, files in os.walk(module_workspace):
            subdir = relpath(dirpath, drake_workspace)
            os.makedirs(join(temp_dir, "drake", subdir))
            for item in files:
                if any([module.startswith("drake.doc"),
                        "images" in subdir,
                        item.endswith(".h")]):
                    dest = join(temp_dir, "drake", subdir, item)
                    if not os.path.exists(dest):
                        os.symlink(join(dirpath, item), dest)


def _generate_doxyfile(*, manifest, out_dir, temp_dir, dot):
    """Creates Doxyfile_CXX from Doxyfile_CXX.in."""
    input_filename = manifest.Rlocation(
        "drake/doc/doxygen_cxx/Doxyfile_CXX.in")
    assert os.path.exists(input_filename)
    output_filename = join(temp_dir, "Doxyfile_CXX")

    cmake_configure_file = manifest.Rlocation(
        "drake/tools/workspace/cmake_configure_file")
    assert os.path.exists(cmake_configure_file)

    definitions = {}
    definitions["INPUT_ROOT"] = temp_dir
    definitions["OUTPUT_DIRECTORY"] = out_dir
    if dot:
        definitions["DOXYGEN_DOT_FOUND"] = "YES"
        definitions["DOXYGEN_DOT_EXECUTABLE"] = dot
    else:
        definitions["DOXYGEN_DOT_FOUND"] = "NO"
        definitions["DOXYGEN_DOT_EXECUTABLE"] = ""

    check_call([
        cmake_configure_file,
        "--input", input_filename,
        "--output", output_filename,
        ] + [
        "-D%s=%s" % (key, value)
        for key, value in definitions.items()
        ])
    assert os.path.exists(output_filename)
    return output_filename


def _is_important_warning(line):
    """Returns true iff the given line of Doxygen output should be promoted to
    a build error.
    """
    # Check for broken links.
    if "unable to resolve reference" in line:
        # Maybe the code specifically wanted to have a broken link.
        if "MODULE_NOT_WRITTEN_YET" in line:
            return False
        # For now, don't error on broken function signatures; only error on the
        # more important links such as section cross-references.  An example:
        # ... unable to resolve ... `AssignRole(SourceId,...)' for ref ...
        if "(" in line:
            return False
        # TODO(#14107) Remove this if-clause once the issue is resolved.
        if "multibody_plant" in line and "df_contact_material" in line:
            return False
        # Broken link.
        return True

    # All good.
    return False


def _postprocess_doxygen_log(original_lines, check_for_errors):
    """If check_for_errors is true, then looks for any important warnings and
    fails-fast if any were found. When in verbose mode, also dumps the log to
    the console.
    """
    # Throw away useless lines.
    #
    # Specifically, we remove stanzas that looks like this:
    #
    #  The following parameters of DiscardZeroGradient(...) are not documented:
    #  parameter 'auto_diff_matrix'
    #
    # The pattern to remove will be the "are not documented" line, followed by
    # some list of one or more parameter names.
    lines = []
    is_ignoring_parameters = False
    for line in original_lines:
        if is_ignoring_parameters:
            if line.startswith("parameter "):
                continue
        is_ignoring_parameters = False
        if line.endswith(" are not documented:"):
            is_ignoring_parameters = True
            continue
        lines.append(line)

    # Print all of the warnings (when requested).
    if verbose():
        for line in lines:
            print("[doxygen] " + line)

    # Check for important warnings (when requested).
    errors = []
    if check_for_errors:
        for line in lines:
            if _is_important_warning(line):
                errors.append(line.replace("warning:", "error:"))
    if errors:
        message = "\n".join(["Problems found by Doxygen:"] + sorted(errors))
        raise RuntimeError(message)


def _build(*, out_dir, temp_dir, modules, quick):
    """Generates into out_dir; writes scratch files into temp_dir.
    As a precondition, both directories must already exist and be empty.
    """
    manifest = runfiles.Create()

    # Find drake's sources.
    drake_workspace = os.path.dirname(os.path.realpath(
        manifest.Rlocation("drake/.bazelproject")))
    assert os.path.exists(drake_workspace), drake_workspace
    assert os.path.exists(join(drake_workspace, "WORKSPACE")), drake_workspace

    # Find doxygen.
    doxygen = manifest.Rlocation("doxygen/doxygen")
    assert os.path.exists(doxygen), doxygen

    # Find dot.
    dot = "/usr/bin/dot"
    assert os.path.exists(dot), dot

    # Configure doxygen.
    doxyfile = _generate_doxyfile(
        manifest=manifest,
        out_dir=out_dir,
        temp_dir=temp_dir,
        dot=(dot if not quick else ""))

    # Prepare our input.
    symlink_input(
        "drake/doc/doxygen_cxx/doxygen_input.txt", temp_dir)
    _symlink_headers(
        drake_workspace=drake_workspace,
        temp_dir=temp_dir,
        modules=modules)

    # Run doxygen.
    check_call([doxygen, doxyfile], cwd=temp_dir)

    # Post-process its log, and check for errors. If we are building only a
    # subset of the docs, we are likely to encounter errors due to the missing
    # sections, so we'll only enable the promotion of warnings to errors when
    # we're building all of the C++ documentation.
    check_for_errors = (len(modules) == 0)
    with open(f"{temp_dir}/doxygen.log", encoding="utf-8") as f:
        lines = [
            line.strip().replace(f"{temp_dir}/", "")
            for line in f.readlines()
        ]
    _postprocess_doxygen_log(lines, check_for_errors)

    # Collect the list of all HTML output files.
    html_files = []
    for dirpath, _, filenames in os.walk(out_dir):
        for filename in filenames:
            if filename.endswith(".html"):
                html_files.append(relpath(join(dirpath, filename), out_dir))

    # Fix the formatting of deprecation text (see drake#15619 for an example).
    perl_statements = [
        # Remove quotes around the removal date.
        r's#(removed from Drake on or after) "(....-..-..)" *\.#\1 \2.#;',
        # Remove all quotes within the explanation text, i.e., the initial and
        # final quotes, as well as internal quotes that might be due to C++
        # multi-line string literals.
        # - The quotes must appear after a "_deprecatedNNNNNN" anchor.
        # - The quotes must appear before a "<br />" end-of-line.
        # Example lines:
        # <dl class="deprecated"><dt><b><a class="el" href="deprecated.html#_deprecated000013">Deprecated:</a></b></dt><dd>"Use RotationMatrix::MakeFromOneVector()." <br />  # noqa
        # <dd><a class="anchor" id="_deprecated000013"></a>"Use RotationMatrix::MakeFromOneVector()." <br />  # noqa
        r'while (s#(?<=_deprecated\d{6}")([^"]*)"(.*?<br)#\1\2#) {};',
    ]
    while html_files:
        # Work in batches of 100, so we don't overflow the argv limit.
        first, html_files = html_files[:100], html_files[100:]
        check_call(["perl", "-pi", "-e", "".join(perl_statements)] + first,
                   cwd=out_dir)

    # The nominal pages to offer for preview.
    return ["", "classes.html", "modules.html"]


if __name__ == '__main__':
    main(build=_build, subdir="doxygen_cxx", description=__doc__.strip(),
         supports_modules=True, supports_quick=True)
