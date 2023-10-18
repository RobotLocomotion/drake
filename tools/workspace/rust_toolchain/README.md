
The bazel @rules_rust toolchain logic contains a lot of load-time options that
make it difficult to pin down exactly what is being fetched. However for Drake,
we want to keep a carefully inventory of what we're using.

To that end, we "vendor" the result of what rust_register_toolchains() would
usually do and commit the information into Drake. That vendored information is
created by the `update.sh` script and lives in the `lock/**` subdirectory tree.

In short, we pin pointers to rust-lang.org downloads for the compiler, standard
library, etc. along with sha256 checksums and generated BUILD wrapper files.

The rustc version is governed by whatever @rules_rust pins to by default.

The `upgrade.py` script is a fully-automatic mechanism to upgrade the pinned
toolchain after an upgrade to @rules_rust. It uses the `upgrade` directory as
a scratch workspace to help bootstrap the process.
