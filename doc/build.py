"""Command-line tool to generate the full drake.mit.edu website contents.

For now, this tool is only intended to be used by Drake's CI tooling.  In the
future, we will enhance and document it for developer use.
"""

import argparse
import os.path
from pathlib import Path
import shlex
import shutil
import subprocess
import urllib.parse

from bazel_tools.tools.python.runfiles import runfiles
import lxml.etree as ET


def _check_call(args):
    print("+ " + " ".join([shlex.quote(x) for x in args]), flush=True)
    proc = subprocess.run(args, stderr=subprocess.STDOUT)
    proc.check_returncode()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__.strip())
    parser.add_argument(
        "--out_dir", type=str, metavar="DIR", required=True,
        help="Output directory. Must be an absolute path and must not exist.")

    args = parser.parse_args()
    out_dir = args.out_dir
    if not os.path.isabs(out_dir):
        parser.error(f"--out_dir={out_dir} is not an absolute path")
    if os.path.exists(out_dir):
        parser.error(f"--out_dir={out_dir} already exists")

    manifest = runfiles.Create()
    gen_sphinx = manifest.Rlocation("drake/bindings/pydrake/doc/gen_sphinx")
    gen_jekyll = manifest.Rlocation("drake/doc/gen_jekyll")
    doxygen = manifest.Rlocation("drake/doc/doxygen")
    for item in [gen_sphinx, gen_jekyll, doxygen]:
        assert os.path.exists(item), item

    _check_call([gen_jekyll, f"--out_dir={out_dir}"])
    _check_call([gen_sphinx, f"--out_dir={out_dir}/pydrake"])
    doxygen_scratch = f"{out_dir}/doxygen_scratch"
    _check_call([doxygen, f"--out_dir={doxygen_scratch}"])
    print(f"+ mv {doxygen_scratch}/html {out_dir}/doxygen_cxx")
    os.rename(f"{doxygen_scratch}/html", f"{out_dir}/doxygen_cxx")
    print(f"+ rm -rf {doxygen_scratch}")
    shutil.rmtree(doxygen_scratch)
    # TODO(jwnimmer-tri) Incorporate the Drake styleguide publication here,
    # instead of having it be a separate pipeline.

    _build_sitemap(out_dir)


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
    assert (root_path.is_absolute(),
            "Path to generated website is not an absolute path")
    paths = root_path.glob("**/*.html")

    XML_NAMESPACE = "http://www.sitemaps.org/schemas/sitemap/0.9"
    ROOT_URL = "https://drake.mit.edu"

    urlset = ET.Element("urlset", xmlns=XML_NAMESPACE)
    for path in sorted(paths):
        relative_path = path.relative_to(root_path)
        url = ET.SubElement(urlset, "url")
        if relative_path.name == "index.html":
            # sitemap.xml should only include canonical urls.
            location = relative_path.parent.as_posix() + "/"
        else:
            location = relative_path.as_posix()
            location = urllib.parse.urljoin(ROOT_URL,
                                            urllib.parse.quote(location))
        loc = ET.SubElement(url, "loc")
        loc.text = location
    sitemap = ET.ElementTree(urlset)
    sitemap.write(os.path.join(site_dir, "sitemap.xml"),
                  encoding="utf-8",
                  pretty_print=True,
                  xml_declaration=True)


if __name__ == '__main__':
    main()
