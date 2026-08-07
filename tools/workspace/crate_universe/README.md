
# Rust dependencies

The purpose of `//tools/workspace/crate_universe` is to provide a central
location to declare Drake's (pinned) dependencies on Rust crates, and the
module extension to load them into our `MODULE.bazel`.

To this end, we use the Cargo dependency resolver from rules_rs:
  https://github.com/hermeticbuild/rules_rs

The Cargo manifest comes from Clarabel.cpp's `rust_wrapper/Cargo.toml`.
The lockfile and `lock/repo_names.bzl` are kept under `lock/` and managed by
`upgrade.py`.

## Upgrading

To upgrade Drake's pinned Rust dependencies, run this tool:
  bazel run //tools/workspace/crate_universe:upgrade
