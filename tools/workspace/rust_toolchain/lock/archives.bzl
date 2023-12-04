# This file is automatically generated by upgrade.py.

ARCHIVES = [
    dict(
        name = "rust_darwin_aarch64__aarch64-apple-darwin__stable",
        build_file = Label("@drake//tools/workspace/rust_toolchain:lock/details/BUILD.rust_darwin_aarch64__aarch64-apple-darwin__stable.bazel"),
        downloads = "[]",
    ),
    dict(
        name = "rust_darwin_aarch64__aarch64-apple-darwin__stable_tools",
        build_file = Label("@drake//tools/workspace/rust_toolchain:lock/details/BUILD.rust_darwin_aarch64__aarch64-apple-darwin__stable_tools.bazel"),
        downloads = json.encode(
            [
                {
                    "sha256": "03a8e33693d4e532368708cff5fafbde71e612c0e5c5f14b5fb80be0d7817a8a",
                    "stripPrefix": "rustc-1.74.0-aarch64-apple-darwin/rustc",
                    "url": [
                        "https://static.rust-lang.org/dist/rustc-1.74.0-aarch64-apple-darwin.tar.gz",
                    ],
                },
                {
                    "sha256": "efa0803220766039dafe2b397e06737a1a001ad0ad5deca160e421f46844d768",
                    "stripPrefix": "clippy-1.74.0-aarch64-apple-darwin/clippy-preview",
                    "url": [
                        "https://static.rust-lang.org/dist/clippy-1.74.0-aarch64-apple-darwin.tar.gz",
                    ],
                },
                {
                    "sha256": "b7e4c2a829bd8bc90101067ba96b71ecd73fe130401b2478b095047bd8acb469",
                    "stripPrefix": "cargo-1.74.0-aarch64-apple-darwin/cargo",
                    "url": [
                        "https://static.rust-lang.org/dist/cargo-1.74.0-aarch64-apple-darwin.tar.gz",
                    ],
                },
                {
                    "sha256": "3c5e6f5ab874d3c338c46f439d5f0112356028284ece0b6643bf0080e9ea5f85",
                    "stripPrefix": "rustfmt-1.74.0-aarch64-apple-darwin/rustfmt-preview",
                    "url": [
                        "https://static.rust-lang.org/dist/rustfmt-1.74.0-aarch64-apple-darwin.tar.gz",
                    ],
                },
                {
                    "sha256": "694b09a509240abcad893868f506d72f83494f4bed879291ebe08fe48c4f8f9b",
                    "stripPrefix": "llvm-tools-1.74.0-aarch64-apple-darwin/llvm-tools-preview",
                    "url": [
                        "https://static.rust-lang.org/dist/llvm-tools-1.74.0-aarch64-apple-darwin.tar.gz",
                    ],
                },
                {
                    "sha256": "5c02396eb7cfe1a3c12b01dffd758cf862c4264df6280727798745b98f245082",
                    "stripPrefix": "rust-std-1.74.0-aarch64-apple-darwin/rust-std-aarch64-apple-darwin",
                    "url": [
                        "https://static.rust-lang.org/dist/rust-std-1.74.0-aarch64-apple-darwin.tar.gz",
                    ],
                },
            ],
        ),
    ),
    dict(
        name = "rust_darwin_x86_64__x86_64-apple-darwin__stable",
        build_file = Label("@drake//tools/workspace/rust_toolchain:lock/details/BUILD.rust_darwin_x86_64__x86_64-apple-darwin__stable.bazel"),
        downloads = "[]",
    ),
    dict(
        name = "rust_darwin_x86_64__x86_64-apple-darwin__stable_tools",
        build_file = Label("@drake//tools/workspace/rust_toolchain:lock/details/BUILD.rust_darwin_x86_64__x86_64-apple-darwin__stable_tools.bazel"),
        downloads = json.encode(
            [
                {
                    "sha256": "3919989c194b2a6674702f0323cac4229d7c6939c933acf4e4451c144f69c8ed",
                    "stripPrefix": "rustc-1.74.0-x86_64-apple-darwin/rustc",
                    "url": [
                        "https://static.rust-lang.org/dist/rustc-1.74.0-x86_64-apple-darwin.tar.gz",
                    ],
                },
                {
                    "sha256": "fbab76d6ab912ca4e31eea9a44db97fff22d373de10cfb9c75c29ed124d42a3f",
                    "stripPrefix": "clippy-1.74.0-x86_64-apple-darwin/clippy-preview",
                    "url": [
                        "https://static.rust-lang.org/dist/clippy-1.74.0-x86_64-apple-darwin.tar.gz",
                    ],
                },
                {
                    "sha256": "f45dec402a07acf072f1f58064cb3d21cd795914182e8260d88fce73f082b577",
                    "stripPrefix": "cargo-1.74.0-x86_64-apple-darwin/cargo",
                    "url": [
                        "https://static.rust-lang.org/dist/cargo-1.74.0-x86_64-apple-darwin.tar.gz",
                    ],
                },
                {
                    "sha256": "4f774c33527884ee2b0ca378112732e0cb1ad558d4958f9673e2a1ef1055b66e",
                    "stripPrefix": "rustfmt-1.74.0-x86_64-apple-darwin/rustfmt-preview",
                    "url": [
                        "https://static.rust-lang.org/dist/rustfmt-1.74.0-x86_64-apple-darwin.tar.gz",
                    ],
                },
                {
                    "sha256": "e724b94dfe302b729ff087fe7e81850c7a3e048993d24f5c063659d3d7d141f7",
                    "stripPrefix": "llvm-tools-1.74.0-x86_64-apple-darwin/llvm-tools-preview",
                    "url": [
                        "https://static.rust-lang.org/dist/llvm-tools-1.74.0-x86_64-apple-darwin.tar.gz",
                    ],
                },
                {
                    "sha256": "ffd3de3b29d324d7c8b8b57569c11bf3749fc6313ec0b2638ef38997bdbdb6fc",
                    "stripPrefix": "rust-std-1.74.0-x86_64-apple-darwin/rust-std-x86_64-apple-darwin",
                    "url": [
                        "https://static.rust-lang.org/dist/rust-std-1.74.0-x86_64-apple-darwin.tar.gz",
                    ],
                },
            ],
        ),
    ),
    dict(
        name = "rust_linux_aarch64__aarch64-unknown-linux-gnu__stable",
        build_file = Label("@drake//tools/workspace/rust_toolchain:lock/details/BUILD.rust_linux_aarch64__aarch64-unknown-linux-gnu__stable.bazel"),
        downloads = "[]",
    ),
    dict(
        name = "rust_linux_aarch64__aarch64-unknown-linux-gnu__stable_tools",
        build_file = Label("@drake//tools/workspace/rust_toolchain:lock/details/BUILD.rust_linux_aarch64__aarch64-unknown-linux-gnu__stable_tools.bazel"),
        downloads = json.encode(
            [
                {
                    "sha256": "8e84e8065f21ea01ede5982869dd61160b1999b17f9a79911979ee936aea0de9",
                    "stripPrefix": "rustc-1.74.0-aarch64-unknown-linux-gnu/rustc",
                    "url": [
                        "https://static.rust-lang.org/dist/rustc-1.74.0-aarch64-unknown-linux-gnu.tar.gz",
                    ],
                },
                {
                    "sha256": "a05c6d21e189dd0824234829229363cdb387375238796a0fb4445dc6cd669555",
                    "stripPrefix": "clippy-1.74.0-aarch64-unknown-linux-gnu/clippy-preview",
                    "url": [
                        "https://static.rust-lang.org/dist/clippy-1.74.0-aarch64-unknown-linux-gnu.tar.gz",
                    ],
                },
                {
                    "sha256": "ab22b5aa6baa622a267f98ef2f1d06dd5a4a95b7ca6cadb0c431d31f1e018251",
                    "stripPrefix": "cargo-1.74.0-aarch64-unknown-linux-gnu/cargo",
                    "url": [
                        "https://static.rust-lang.org/dist/cargo-1.74.0-aarch64-unknown-linux-gnu.tar.gz",
                    ],
                },
                {
                    "sha256": "f6aca2448687de82323724857e36a5104fe7503ffaae2f13f98bcaf6168de251",
                    "stripPrefix": "rustfmt-1.74.0-aarch64-unknown-linux-gnu/rustfmt-preview",
                    "url": [
                        "https://static.rust-lang.org/dist/rustfmt-1.74.0-aarch64-unknown-linux-gnu.tar.gz",
                    ],
                },
                {
                    "sha256": "b45921e2077c8542af2f258cce5ad25f896ffac4804e5ad2de957e573a16b183",
                    "stripPrefix": "llvm-tools-1.74.0-aarch64-unknown-linux-gnu/llvm-tools-preview",
                    "url": [
                        "https://static.rust-lang.org/dist/llvm-tools-1.74.0-aarch64-unknown-linux-gnu.tar.gz",
                    ],
                },
                {
                    "sha256": "56df7a51381bdf38ceba057c93581d00aab4619d78974bca9f47cbc49aa8497b",
                    "stripPrefix": "rust-std-1.74.0-aarch64-unknown-linux-gnu/rust-std-aarch64-unknown-linux-gnu",
                    "url": [
                        "https://static.rust-lang.org/dist/rust-std-1.74.0-aarch64-unknown-linux-gnu.tar.gz",
                    ],
                },
            ],
        ),
    ),
    dict(
        name = "rust_linux_x86_64__x86_64-unknown-linux-gnu__stable",
        build_file = Label("@drake//tools/workspace/rust_toolchain:lock/details/BUILD.rust_linux_x86_64__x86_64-unknown-linux-gnu__stable.bazel"),
        downloads = "[]",
    ),
    dict(
        name = "rust_linux_x86_64__x86_64-unknown-linux-gnu__stable_tools",
        build_file = Label("@drake//tools/workspace/rust_toolchain:lock/details/BUILD.rust_linux_x86_64__x86_64-unknown-linux-gnu__stable_tools.bazel"),
        downloads = json.encode(
            [
                {
                    "sha256": "358422396f3ff2a073f6fce66ca5aad9ae0596452711f6728c87698846c74e2a",
                    "stripPrefix": "rustc-1.74.0-x86_64-unknown-linux-gnu/rustc",
                    "url": [
                        "https://static.rust-lang.org/dist/rustc-1.74.0-x86_64-unknown-linux-gnu.tar.gz",
                    ],
                },
                {
                    "sha256": "7a6066568044fa0eaadda9f52eb9cb1a72973bd45931871c77ada1a6d8387766",
                    "stripPrefix": "clippy-1.74.0-x86_64-unknown-linux-gnu/clippy-preview",
                    "url": [
                        "https://static.rust-lang.org/dist/clippy-1.74.0-x86_64-unknown-linux-gnu.tar.gz",
                    ],
                },
                {
                    "sha256": "38451abcf728c8583cba29dbd74debf56ce585dcc829ac7b03ccf94a563b8ddf",
                    "stripPrefix": "cargo-1.74.0-x86_64-unknown-linux-gnu/cargo",
                    "url": [
                        "https://static.rust-lang.org/dist/cargo-1.74.0-x86_64-unknown-linux-gnu.tar.gz",
                    ],
                },
                {
                    "sha256": "6351f586dd385e55945175ba8e8085fbf4974c27d9d3bb02e6e6ea01de7900f5",
                    "stripPrefix": "rustfmt-1.74.0-x86_64-unknown-linux-gnu/rustfmt-preview",
                    "url": [
                        "https://static.rust-lang.org/dist/rustfmt-1.74.0-x86_64-unknown-linux-gnu.tar.gz",
                    ],
                },
                {
                    "sha256": "687c306cb5e68f1efbeb8515c1f7b8563fdee6c88b1c1f574fafc61dcb67bd98",
                    "stripPrefix": "llvm-tools-1.74.0-x86_64-unknown-linux-gnu/llvm-tools-preview",
                    "url": [
                        "https://static.rust-lang.org/dist/llvm-tools-1.74.0-x86_64-unknown-linux-gnu.tar.gz",
                    ],
                },
                {
                    "sha256": "798b3243d9236e4dc5d43f6b186333cd30c04926b2229568d1fc0f0eb432507f",
                    "stripPrefix": "rust-std-1.74.0-x86_64-unknown-linux-gnu/rust-std-x86_64-unknown-linux-gnu",
                    "url": [
                        "https://static.rust-lang.org/dist/rust-std-1.74.0-x86_64-unknown-linux-gnu.tar.gz",
                    ],
                },
            ],
        ),
    ),
]
