
# Rust dependencies

The purpose of `//tools/workspace/crate_universe` is to provide a central
location to declare Drake's (pinned) dependencies on Rust crates, and the
module extension to load them into our `MODULE.bazel`.

To this end, we use the Cargo dependency resolver from rules_rs:
  https://github.com/hermeticbuild/rules_rs

The Cargo manifest, lockfile, and `lock/repo_names.bzl` are kept under `lock/`.
Keep `lock/Cargo.toml` synchronized with Clarabel.cpp's
`rust_wrapper/Cargo.toml`.
The lockfile and `lock/repo_names.bzl` are managed by `upgrade.sh`.
The existing files in `lock/details/**` remain in the repository but are ignored
by Bazel and are not used to resolve or build Rust dependencies.

## Upgrading

To upgrade Drake's pinned Rust dependencies, run this tool:
  tools/workspace/crate_universe/upgrade.sh
