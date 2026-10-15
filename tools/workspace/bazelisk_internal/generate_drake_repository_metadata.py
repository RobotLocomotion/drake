"""Respells the setup/ubuntu/packages.json URLs for Bazelisk into the metadata
format necessary for mirror_to_s3."""

import json
from pathlib import Path
import sys


def _main():
    (input_path, output_path) = sys.argv[1:]

    # Load the json details from our `setup` data.
    packages = json.loads(Path(input_path).read_text(encoding="utf-8"))
    bazelisk_packages = [p for p in packages if p["name"] == "bazelisk"]
    assert bazelisk_packages

    # Respell the json format.
    downloads = []
    for package in bazelisk_packages:
        downloads.append(
            {
                "urls": package["urls"],
                "sha256": package["sha256"],
            }
        )

    # Write the output.
    result = {
        "name": "bazelisk_debs_internal",
        "repository_rule_type": "scripted",
        "downloads": downloads,
    }
    Path(output_path).write_text(json.dumps(result), encoding="utf-8")


assert __name__ == "__main__"
_main()
