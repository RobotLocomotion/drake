"""Reports summary data related to Drake's repository rules.  This
implementation uses Bazel command-line actions so is suitable only
for manual use, not any build rules or test automation.
"""

import json
from pathlib import Path

from python import runfiles


def read_repository_metadata():
    """Returns a dict of {repository_name => details_dict}."""
    result = {}
    manifest = runfiles.Create()
    inventory_path = Path(
        manifest.Rlocation(
            "drake/tools/workspace/drake_repository_metadata_inventory.txt",
        )
    )
    for line in inventory_path.read_text(encoding="utf-8").splitlines():
        canonical_repo_name, json_name = line.split("/")
        assert json_name == "drake_repository_metadata.json"
        apparent_repo_name = canonical_repo_name.split("+")[-1]
        json_path = Path(
            manifest.Rlocation(f"{apparent_repo_name}/{json_name}")
        )
        data = json.loads(json_path.read_text(encoding="utf-8"))
        result[data["name"]] = data

    # Add 'magic' metadata for repositories that don't/can't generate it the
    # usual way.
    result["crate_universe"] = {
        "repository_rule_type": "scripted",
        "upgrade_script": "upgrade.sh",
        # Downloads are associated with individual "crate__..." repositories.
        "downloads": {},
    }

    return result
