"""Command-line tool to generate the full drake.mit.edu website contents.

For instructions, see https://drake.mit.edu/documentation_instructions.html.
"""

import os.path
from pathlib import Path
import sys
import urllib.parse

from bazel_tools.tools.python.runfiles import runfiles
import lxml.etree as ET

from drake.doc.defs import check_call, main


def _build(*, out_dir, temp_dir, quick, modules):
    """Callback function that implements the bulk of main().
    Generates into out_dir; writes scratch files into temp_dir.
    As a precondition, both directories must already exist and be empty.

    If provided, the given modules can be either API reference modules such as
    "drake.math" (C++) or "pydrake.math" (Python), or else the name of website
    sections such as "pages", "styleguide", "pydrake", "doxygen_cxx", or etc.
    """
    # Find all of our helper tools.
    manifest = runfiles.Create()
    pages_build = manifest.Rlocation("drake/doc/pages")
    styleguide_build = manifest.Rlocation("drake/doc/styleguide/build")
    pydrake_build = manifest.Rlocation("drake/doc/pydrake/build")
    doxygen_build = manifest.Rlocation("drake/doc/doxygen_cxx/build")
    for item in [pages_build, styleguide_build, pydrake_build, doxygen_build]:
        assert item and os.path.exists(item), item

    # Figure out which modules to ask for from each helper tool.
    do_pages = True
    do_styleguide = True
    do_pydrake = True
    do_doxygen = True
    do_sitemap = True
    pydrake_modules = []
    doxygen_modules = []
    if modules:
        do_pages = False
        do_styleguide = False
        do_pydrake = False
        do_doxygen = False
        do_sitemap = False
    for module in modules:
        if module in ["pages"]:
            do_pages = True
        elif module in ["styleguide", "cppguide", "pyguide"]:
            do_styleguide = True
        elif module in ["pydrake"]:
            do_pydrake = True
        elif module in ["doxygen_cxx", "doxygen", "cxx"]:
            do_doxygen = True
        elif module in ["sitemap"]:
            do_sitemap = True
        elif module.startswith("pydrake."):
            do_pydrake = True
            pydrake_modules.append(module)
        elif module.startswith("drake."):
            do_doxygen = True
            doxygen_modules.append(module)
        else:
            print(f"error: Unknown module '{module}'")
            sys.exit(1)

    # Invoke all of our helper tools.
    if do_pages:
        check_call([pages_build, f"--out_dir={out_dir}"])
    if do_styleguide:
        check_call([styleguide_build, f"--out_dir={out_dir}/styleguide"])
    if do_pydrake:
        check_call([pydrake_build, f"--out_dir={out_dir}/pydrake"]
                   + pydrake_modules)
    if do_doxygen:
        maybe_quick = ["--quick"] if quick else []
        check_call([doxygen_build, f"--out_dir={out_dir}/doxygen_cxx"]
                   + doxygen_modules + maybe_quick)
    if do_sitemap:
        _build_sitemap(out_dir)

    # The filenames to suggest as the starting point for preview.
    result = []
    result.append("") if do_pages else None
    result.append("styleguide/cppguide.html") if do_styleguide else None
    result.append("styleguide/pyguide.html") if do_styleguide else None
    result.append("pydrake/") if do_pydrake else None
    result.append("doxygen_cxx/") if do_doxygen else None
    return result


def _build_sitemap(site_dir: str) -> None:
    """Builds a minimal sitemap.xml for drake.mit.edu.

    This helps Google, Bing, and other search engines decide which pages on the
    generated drake.mit.edu site should be crawled, and helps determine the
    canonical version of each page.

    https://developers.google.com/search/docs/advanced/sitemaps/build-sitemap

    Args:
        site_dir: The absolute path to the root directory of the generated
          website and the directory to which the built sitemap.xml will be
          written.

    Raises:
        OSError: If the directory to which site_dir refers is not readable or
          writable.
    """

    print("Building sitemap.xml...")
    root_path = Path(site_dir)
    assert root_path.is_absolute(), \
        "Path to generated website is not an absolute path"
    paths = root_path.glob("**/*.html")

    XML_NAMESPACE = "http://www.sitemaps.org/schemas/sitemap/0.9"
    ROOT_URL = "https://drake.mit.edu"

    urlset = ET.Element("urlset", xmlns=XML_NAMESPACE)
    for path in sorted(paths):
        relative_path = path.relative_to(root_path)
        url = ET.SubElement(urlset, "url")
        if relative_path.name == "index.html":
            # sitemap.xml should only include canonical urls.
            relative_location = relative_path.parent.as_posix() + "/"
        else:
            relative_location = relative_path.as_posix()
        location = urllib.parse.urljoin(ROOT_URL,
                                        urllib.parse.quote(relative_location))
        loc = ET.SubElement(url, "loc")
        loc.text = location
    sitemap = ET.ElementTree(urlset)
    sitemap.write(os.path.join(site_dir, "sitemap.xml"),
                  encoding="utf-8",
                  pretty_print=True,
                  xml_declaration=True)


if __name__ == '__main__':
    main(build=_build, subdir="", description=__doc__.strip(),
         supports_modules=True, supports_quick=True)
