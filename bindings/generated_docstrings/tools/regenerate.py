from pathlib import Path
import shutil

from python import runfiles


def main():
    manifest = runfiles.Create()

    # Load the list of filenames.
    resource_path = "drake/bindings/generated_docstrings/filenames.txt"
    filenames_path = Path(manifest.Rlocation(resource_path))
    filenames = filenames_path.read_text(encoding="utf-8").splitlines()

    # We'll copy the files back into the source tree at `generated_docstrings`.
    output_dir = Path(__file__).resolve(strict=True).parent.parent

    # Copy ...docstrings/gen/{filename} to ...docstrings/{filename} (the former
    # are runfile resources, the latter are in the source tree).
    gen_dir = "drake/bindings/generated_docstrings/gen"
    for filename in filenames:
        input_path = Path(manifest.Rlocation(f"{gen_dir}/{filename}"))
        output_path = output_dir / filename
        shutil.copyfile(input_path, output_path)


assert __name__ == "__main__"
main()
