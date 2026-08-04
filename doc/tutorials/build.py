"""Command-line tool to generate Drake's rendered tutorials.

For instructions, see https://drake.mit.edu/documentation_instructions.html.
"""

import os
from pathlib import Path
import sys

from python import runfiles

from doc.defs import (
    check_call,
    main,
    perl_cleanup_html_output,
    symlink_input,
)

# These versions match what's shipped in Ubuntu 24.04 Noble, which is what our
# CI documentation build uses when running jupyter-nbconvert.
_MATHJAX_URL = "https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.9/MathJax.js?config=TeX-AMS-MML_HTMLorMML"  # noqa
_REQUIREJS_URL = (
    "https://cdnjs.cloudflare.com/ajax/libs/require.js/2.3.6/require.min.js"
)

# These tutorials cannot be executed during the documentation build.
_NO_EXECUTE = (
    # Requires deepnote.
    "licensed_solvers_deepnote",
)


def _build(*, out_dir, temp_dir, modules):
    """Generates into out_dir; writes scratch files into temp_dir.
    As a precondition, both directories must already exist and be empty.
    If modules are provided, only generate those modules and their children.
    """
    assert len(os.listdir(temp_dir)) == 0
    assert len(os.listdir(out_dir)) == 0

    nbconvert = "/usr/bin/jupyter-nbconvert"
    if not os.path.isfile(nbconvert):
        print(
            "Please re-run 'setup/install_prereqs' with the '--developer' flag"
        )
        sys.exit(1)

    # Find Drake's nbconvert templates.
    manifest = runfiles.Create()
    templates_dir = Path(
        manifest.Rlocation(
            "drake/doc/tutorials/templates/drake_style/conf.json"
        )
    ).parent.parent

    # Create a hermetic copy of our input.  This helps ensure that only files
    # listed in BUILD.bazel will render onto the website.
    symlink_input(
        "drake/doc/tutorials/notebooks.txt",
        temp_dir,
        strip_prefix=["drake/doc/"],
    )
    input_dir = Path(temp_dir) / "drake/tutorials"
    all_tutorials = sorted(["tutorials." + x.stem for x in input_dir.iterdir()])

    # Process the command-line request for which tutorials to document.
    if not modules:
        modules_to_document = set(all_tutorials)
    else:
        modules_to_document = set()
        for x in modules:
            if x in all_tutorials:
                modules_to_document.add(x)
            else:
                print(f"error: Unknown tutorial '{x}'")
                sys.exit(1)

    # Run the documentation generator.
    for module in modules_to_document:
        assert module.startswith("tutorials.")
        name = module[len("tutorials.") :]
        check_call(
            [
                nbconvert,
                str(input_dir / f"{name}.ipynb"),
                "--to=html",
                f"--TemplateExporter.extra_template_basedirs={templates_dir}",
                "--template=drake_style",
                f"--HTMLExporter.mathjax_url={_MATHJAX_URL}",
                f"--HTMLExporter.require_js_url={_REQUIREJS_URL}",
                f"--output={out_dir}/{name}.html",
            ]
            + (["--execute"] if name not in _NO_EXECUTE else [])
        )

    # Tidy up.
    perl_cleanup_html_output(
        out_dir=out_dir,
        extra_perl_statements=[
            # Replace links to ipynb files with links to html, instead.
            r's#(href="\./[^.]*?\.)ipynb#$1html#g;',
        ],
    )

    # The filename to suggest as the starting point for preview; in this case,
    # it's an empty filename (i.e., the index page).
    return [""]


if __name__ == "__main__":
    os.environ["PYTHONPATH"] = ":".join(sys.path)
    main(
        build=_build,
        subdir="tutorials",
        description=__doc__.strip(),
        supports_modules=True,
    )
