import argparse
import json


def _create_models_header(json_data):
    struct_type = "drake::multibody::PackageMap::RemotePackageParams"
    result = [
        "#pragma once",
        "namespace {",
        "auto GetDrakeModelsParams() {",
        f"  return {struct_type}{{",
        "    .urls = {",
    ]
    for url in json_data["urls"]:
        result.append(f'      "{url}",')
    result += [
        "    },",
        f'    .sha256 = "{json_data["sha256"]},"',
        "  };",
        "}",
        "}  //namaespace",
    ]
    # return str(json_data)
    return "\n".join(result) + "\n"


def _main():
    parser = argparse.ArgumentParser(
        description="Test utility for Meshcat websockets")
    parser.add_argument(
        "--input", metavar='FILE',
        help="JSON repository metadata")
    parser.add_argument(
        "--output", metavar='FILE',
        help="C++ header filename")
    args = parser.parse_args()
    with open(args.input, "r", encoding="utf-8") as f:
        json_data = json.load(f)
    header = _create_models_header(json_data)
    with open(args.output, "w", encoding="utf-8") as f:
        f.write(header)
    return 0


assert __name__ == "__main__"
_main()
