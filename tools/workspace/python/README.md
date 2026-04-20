When changing the set of supported Python versions (or the default Python
version on macOS), the following files need attention:

- `drake/CMakeLists.txt` near `find_package(Python <min_version> ...)`,
- `drake/MODULE.bazel` near `PYTHON_VERSIONS`,
- `drake/doc/_pages/installation.md` in the support matrix column for `Python`,
- `drake/doc/_pages/from_source.md` in the support matrix column for `Python`,
- `drake/doc/_release-notes/end_of_support.md` if dropping support,
- `drake/setup/mac/Brewfile-binary`,
- `drake/setup/mac/install_prereqs_binary.sh` near the `pip3.##` and
  `python3.##` program names,
- `drake/setup/mac/Brewfile-developer`,
- `drake/setup/mac/install_prereqs_user_environment.sh` near
  `gen/python_version.txt`,
- `drake/setup/python/mypy/BUILD.bazel` near `--python-version`,
- `drake/setup/python/pyproject.toml` near `requires-python`,
- `drake/tools/wheel/image/setup.py` near `python_requires`,
- `drake/tools/wheel/wheel_builder/linux.py` near `targets`,
- `drake/tools/wheel/wheel_builder/macos.py` near `targets`
- `drake/tools/workspace/python/repository.bzl` near `macos_interpreter_path`,
- `drake/tools/workspace/python/venv_upgrade` near `python_versions`.
