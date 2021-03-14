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
    parser.add_argument(
        "--sitemap", action="store_true",
        help="Generate a sitemap.xml in the root of the output directory")

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

    if args.sitemap:
        # Generate a minimal sitemap.xml
        # https://developers.google.com/search/docs/advanced/sitemaps/build-sitemap # noqa
        import lxml.etree as ET

        print("Generating sitemap.xml...")
        root_path = Path(out_dir)
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
        sitemap.write("sitemap.xml",
                      encoding="utf-8",
                      pretty_print=True,
                      xml_declaration=True)


if __name__ == '__main__':
    main()
