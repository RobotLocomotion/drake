"""Command-line tool to generate Drake's C++ API reference.

For instructions, see https://drake.mit.edu/documentation_instructions.html.
"""

from glob import glob
import os
from os.path import join
from pathlib import Path
import shutil
import stat
import sys

from python import runfiles

from doc.defs import (
    check_call,
    main,
    perl_cleanup_html_output,
    symlink_input,
    verbose,
)


def _cull_skipped_headers(*, temp_dir, modules):
    """Removes the header file symlinks for any modules beyond the requested
    list of `modules`, but never removes any headers under `doc/**`.
    """
    # It's easier to deal with directory paths than dotted strings.
    module_paths = [Path(x.replace(".", "/")) for x in modules]

    # Never remove any headers under `doc/**`.
    module_paths.append(Path("drake/doc"))

    # Validate the list of modules.
    for module_path in module_paths:
        top = str(module_path.parents[-2])
        if top != "drake":
            print(f"error: Modules must start with 'drake', not {top!r}")
            sys.exit(1)
        if not (temp_dir / module_path).is_dir():
            print(f"error: Unknown module {module_path}")
            sys.exit(1)

    # Cull the directories as requested.
    for dirpath, _, filenames in (temp_dir / Path("drake")).walk():
        subdir = dirpath.relative_to(temp_dir)
        is_wanted = any(
            [
                str(subdir).startswith(str(module_path))
                for module_path in module_paths
            ]
        )
        if is_wanted:
            continue
        for filename in filenames:
            (dirpath / filename).unlink()


def _generate_doxyfile(*, manifest, out_dir, temp_dir, dot):
    """Creates Doxyfile_CXX from Doxyfile_CXX.in."""
    input_filename = manifest.Rlocation("drake/doc/doxygen_cxx/Doxyfile_CXX.in")
    assert os.path.exists(input_filename)
    output_filename = join(temp_dir, "Doxyfile_CXX")

    cmake_configure_file = manifest.Rlocation(
        "drake/tools/workspace/cmake_configure_file"
    )
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

    check_call(
        [
            cmake_configure_file,
            "--input",
            input_filename,
            "--output",
            output_filename,
        ]
        + ["-D%s=%s" % (key, value) for key, value in definitions.items()]
    )
    assert os.path.exists(output_filename)
    return output_filename


def _generate_system_doxygen_wrapper(*, temp_dir):
    # This matches Doxyfile_CXX.
    script = "drake/doc/doxygen_cxx/system_doxygen.py"
    content = f'exec {sys.executable} {script} "$@"'
    wrapper_path = f"{temp_dir}/system_doxygen.sh"
    Path(wrapper_path).write_text(content, encoding="utf-8")
    os.chmod(wrapper_path, stat.S_IRUSR | stat.S_IXUSR)


def _generate_doxygen_header(*, doxygen, temp_dir):
    """Creates Drake's header.html based on a patch to Doxygen's default
    header template.
    """
    # This matches Doxyfile_CXX.
    header_path = f"{temp_dir}/drake/doc/doxygen_cxx/header.html"

    # Extract the default templates from the Doxygen binary. We only want the
    # header, but it forces us to create all three in this exact order.
    scratch_files = [
        "header.html.orig",
        "footer.html.orig",
        "customdoxygen.css.orig",
    ]
    check_call([doxygen, "-w", "html"] + scratch_files, cwd=temp_dir)
    shutil.copy(f"{temp_dir}/header.html.orig", header_path)
    for orig in scratch_files:
        os.remove(f"{temp_dir}/{orig}")

    # Apply our patch.
    patch_file = f"{header_path}.patch"
    check_call(["/usr/bin/patch", header_path, patch_file])


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
        # Broken link.
        return True

    # Check for failing `@param` commands (e.g., typos in the identifier name).
    if "@param" in line:
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

    # Find doxygen.
    doxygen = manifest.Rlocation("doxygen_internal/doxygen")
    assert os.path.exists(doxygen), doxygen

    # Find dot.
    dot = "/usr/bin/dot"
    assert os.path.exists(dot), dot

    # Configure doxygen.
    doxyfile = _generate_doxyfile(
        manifest=manifest,
        out_dir=out_dir,
        temp_dir=temp_dir,
        dot=(dot if not quick else ""),
    )
    _generate_system_doxygen_wrapper(temp_dir=temp_dir)

    # Prepare our input.
    symlink_input("drake/doc/doxygen_cxx/doxygen_input.txt", temp_dir)
    _generate_doxygen_header(
        doxygen=doxygen,
        temp_dir=temp_dir,
    )
    if len(modules) > 0:
        _cull_skipped_headers(
            temp_dir=temp_dir,
            modules=modules,
        )

    # Run doxygen.
    check_call([doxygen, doxyfile], cwd=temp_dir)

    # Post-process its log, and check for errors. If we are building only a
    # subset of the docs, we are likely to encounter errors due to the missing
    # sections, so we'll only enable the promotion of warnings to errors when
    # we're building all of the C++ documentation.
    check_for_errors = len(modules) == 0
    with open(f"{temp_dir}/doxygen.log", encoding="utf-8") as f:
        lines = [
            line.strip().replace(f"{temp_dir}/", "") for line in f.readlines()
        ]
    _postprocess_doxygen_log(lines, check_for_errors)

    extra_perl_statements = [
        # Fix the formatting of deprecation text (see #15619 for an example).
        # (1) Remove quotes around the removal date.
        r's#(removed from Drake on or after) "(....-..-..)" *\.#\1 \2.#;',
        # (2) Remove all quotes within the explanation text, i.e., the initial
        # and final quotes, as well as internal quotes that might be due to C++
        # multi-line string literals.
        # - The quotes must appear after a "_deprecatedNNNNNN" anchor.
        # - The quotes must appear before a "<br />" end-of-line.
        # Example lines:
        # <dl class="deprecated"><dt><b><a class="el" href="deprecated.html#_deprecated000013">Deprecated:</a></b></dt><dd>"Use RotationMatrix::MakeFromOneVector()." <br />  # noqa
        # <dd><a class="anchor" id="_deprecated000013"></a>"Use RotationMatrix::MakeFromOneVector()." <br />  # noqa
        r'while (s#(?<=_deprecated\d{6}")([^"]*)"(.*?<br)#\1\2#) {};',
    ]
    perl_cleanup_html_output(
        out_dir=out_dir, extra_perl_statements=extra_perl_statements
    )

    # Remove extraneous Doxygen build files.
    files_to_remove = glob(join(out_dir, "*.md5"))
    files_to_remove += glob(join(out_dir, "*.map"))
    for i in files_to_remove:
        try:
            os.remove(i)
        except IOError:
            pass

    # The nominal pages to offer for preview.
    return ["", "classes.html", "modules.html"]


if __name__ == "__main__":
    main(
        build=_build,
        subdir="doxygen_cxx",
        description=__doc__.strip(),
        supports_modules=True,
        supports_quick=True,
    )
