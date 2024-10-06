# This file is automatically generated by upgrade.sh.
ARCHIVES = [
    dict(
        name = "crate__amd-0.2.2",
        sha256 = "a679e001575697a3bd195813feb57a4718ecc08dc194944015cbc5f6213c2b96",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/amd/0.2.2/download"],
        strip_prefix = "amd-0.2.2",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.amd-0.2.2.bazel"),
    ),
    dict(
        name = "crate__autocfg-1.4.0",
        sha256 = "ace50bade8e6234aa140d9a2f552bbee1db4d353f69b8217bc503490fc1a9f26",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/autocfg/1.4.0/download"],
        strip_prefix = "autocfg-1.4.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.autocfg-1.4.0.bazel"),
    ),
    dict(
        name = "crate__blas-0.22.0",
        sha256 = "ae980f75c3215bfe8203c349b28149b0f4130a262e072913ccb55f877cd239dc",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/blas/0.22.0/download"],
        strip_prefix = "blas-0.22.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.blas-0.22.0.bazel"),
    ),
    dict(
        name = "crate__blas-sys-0.7.1",
        sha256 = "13b1b279ceb25d7c4faaea95a5f7addbe7d8c34f9462044bd8e630cebcfc2440",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/blas-sys/0.7.1/download"],
        strip_prefix = "blas-sys-0.7.1",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.blas-sys-0.7.1.bazel"),
    ),
    dict(
        name = "crate__cfg-if-1.0.0",
        sha256 = "baf1de4339761588bc0619e3cbc0120ee582ebb74b53b4efbf79117bd2da40fd",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/cfg-if/1.0.0/download"],
        strip_prefix = "cfg-if-1.0.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.cfg-if-1.0.0.bazel"),
    ),
    dict(
        name = "crate__clarabel-0.9.0",
        patches = [
            "@drake//tools/workspace/crate_universe:patches/clarabel_blas.patch",
        ],
        sha256 = "83e62eacd93b899251364a22bd4dca0f293f8175d05311e42bbf6dbb5edcc762",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/clarabel/0.9.0/download"],
        strip_prefix = "clarabel-0.9.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.clarabel-0.9.0.bazel"),
    ),
    dict(
        name = "crate__darling-0.14.4",
        sha256 = "7b750cb3417fd1b327431a470f388520309479ab0bf5e323505daf0290cd3850",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/darling/0.14.4/download"],
        strip_prefix = "darling-0.14.4",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.darling-0.14.4.bazel"),
    ),
    dict(
        name = "crate__darling_core-0.14.4",
        sha256 = "109c1ca6e6b7f82cc233a97004ea8ed7ca123a9af07a8230878fcfda9b158bf0",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/darling_core/0.14.4/download"],
        strip_prefix = "darling_core-0.14.4",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.darling_core-0.14.4.bazel"),
    ),
    dict(
        name = "crate__darling_macro-0.14.4",
        sha256 = "a4aab4dbc9f7611d8b55048a3a16d2d010c2c8334e46304b40ac1cc14bf3b48e",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/darling_macro/0.14.4/download"],
        strip_prefix = "darling_macro-0.14.4",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.darling_macro-0.14.4.bazel"),
    ),
    dict(
        name = "crate__derive_builder-0.11.2",
        sha256 = "d07adf7be193b71cc36b193d0f5fe60b918a3a9db4dad0449f57bcfd519704a3",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/derive_builder/0.11.2/download"],
        strip_prefix = "derive_builder-0.11.2",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.derive_builder-0.11.2.bazel"),
    ),
    dict(
        name = "crate__derive_builder_core-0.11.2",
        sha256 = "1f91d4cfa921f1c05904dc3c57b4a32c38aed3340cce209f3a6fd1478babafc4",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/derive_builder_core/0.11.2/download"],
        strip_prefix = "derive_builder_core-0.11.2",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.derive_builder_core-0.11.2.bazel"),
    ),
    dict(
        name = "crate__derive_builder_macro-0.11.2",
        sha256 = "8f0314b72bed045f3a68671b3c86328386762c93f82d98c65c3cb5e5f573dd68",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/derive_builder_macro/0.11.2/download"],
        strip_prefix = "derive_builder_macro-0.11.2",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.derive_builder_macro-0.11.2.bazel"),
    ),
    dict(
        name = "crate__either-1.13.0",
        sha256 = "60b1af1c220855b6ceac025d3f6ecdd2b7c4894bfe9cd9bda4fbb4bc7c0d4cf0",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/either/1.13.0/download"],
        strip_prefix = "either-1.13.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.either-1.13.0.bazel"),
    ),
    dict(
        name = "crate__enum_dispatch-0.3.13",
        sha256 = "aa18ce2bc66555b3218614519ac839ddb759a7d6720732f979ef8d13be147ecd",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/enum_dispatch/0.3.13/download"],
        strip_prefix = "enum_dispatch-0.3.13",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.enum_dispatch-0.3.13.bazel"),
    ),
    dict(
        name = "crate__equivalent-1.0.1",
        sha256 = "5443807d6dff69373d433ab9ef5378ad8df50ca6298caf15de6e52e24aaf54d5",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/equivalent/1.0.1/download"],
        strip_prefix = "equivalent-1.0.1",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.equivalent-1.0.1.bazel"),
    ),
    dict(
        name = "crate__fnv-1.0.7",
        sha256 = "3f9eec918d3f24069decb9af1554cad7c880e2da24a9afd88aca000531ab82c1",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/fnv/1.0.7/download"],
        strip_prefix = "fnv-1.0.7",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.fnv-1.0.7.bazel"),
    ),
    dict(
        name = "crate__hashbrown-0.15.0",
        sha256 = "1e087f84d4f86bf4b218b927129862374b72199ae7d8657835f1e89000eea4fb",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/hashbrown/0.15.0/download"],
        strip_prefix = "hashbrown-0.15.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.hashbrown-0.15.0.bazel"),
    ),
    dict(
        name = "crate__ident_case-1.0.1",
        sha256 = "b9e0384b61958566e926dc50660321d12159025e767c18e043daf26b70104c39",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/ident_case/1.0.1/download"],
        strip_prefix = "ident_case-1.0.1",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.ident_case-1.0.1.bazel"),
    ),
    dict(
        name = "crate__indexmap-2.6.0",
        sha256 = "707907fe3c25f5424cce2cb7e1cbcafee6bdbe735ca90ef77c29e84591e5b9da",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/indexmap/2.6.0/download"],
        strip_prefix = "indexmap-2.6.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.indexmap-2.6.0.bazel"),
    ),
    dict(
        name = "crate__itertools-0.11.0",
        sha256 = "b1c173a5686ce8bfa551b3563d0c2170bf24ca44da99c7ca4bfdab5418c3fe57",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/itertools/0.11.0/download"],
        strip_prefix = "itertools-0.11.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.itertools-0.11.0.bazel"),
    ),
    dict(
        name = "crate__itoa-1.0.11",
        sha256 = "49f1f14873335454500d59611f1cf4a4b0f786f9ac11f4312a78e4cf2566695b",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/itoa/1.0.11/download"],
        strip_prefix = "itoa-1.0.11",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.itoa-1.0.11.bazel"),
    ),
    dict(
        name = "crate__lapack-0.19.0",
        sha256 = "ad676a6b4df7e76a9fd80a0c50c619a3948d6105b62a0ab135f064d99c51d207",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/lapack/0.19.0/download"],
        strip_prefix = "lapack-0.19.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.lapack-0.19.0.bazel"),
    ),
    dict(
        name = "crate__lapack-sys-0.14.0",
        sha256 = "447f56c85fb410a7a3d36701b2153c1018b1d2b908c5fbaf01c1b04fac33bcbe",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/lapack-sys/0.14.0/download"],
        strip_prefix = "lapack-sys-0.14.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.lapack-sys-0.14.0.bazel"),
    ),
    dict(
        name = "crate__lazy_static-1.5.0",
        sha256 = "bbd2bcb4c963f2ddae06a2efc7e9f3591312473c50c6685e1f298068316e66fe",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/lazy_static/1.5.0/download"],
        strip_prefix = "lazy_static-1.5.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.lazy_static-1.5.0.bazel"),
    ),
    dict(
        name = "crate__libc-0.2.159",
        sha256 = "561d97a539a36e26a9a5fad1ea11a3039a67714694aaa379433e580854bc3dc5",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/libc/0.2.159/download"],
        strip_prefix = "libc-0.2.159",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.libc-0.2.159.bazel"),
    ),
    dict(
        name = "crate__memchr-2.7.4",
        sha256 = "78ca9ab1a0babb1e7d5695e3530886289c18cf2f87ec19a575a0abdce112e3a3",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/memchr/2.7.4/download"],
        strip_prefix = "memchr-2.7.4",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.memchr-2.7.4.bazel"),
    ),
    dict(
        name = "crate__num-complex-0.4.6",
        sha256 = "73f88a1307638156682bada9d7604135552957b7818057dcef22705b4d509495",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/num-complex/0.4.6/download"],
        strip_prefix = "num-complex-0.4.6",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.num-complex-0.4.6.bazel"),
    ),
    dict(
        name = "crate__num-traits-0.2.19",
        sha256 = "071dfc062690e90b734c0b2273ce72ad0ffa95f0c74596bc250dcfd960262841",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/num-traits/0.2.19/download"],
        strip_prefix = "num-traits-0.2.19",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.num-traits-0.2.19.bazel"),
    ),
    dict(
        name = "crate__once_cell-1.19.0",
        sha256 = "3fdb12b2476b595f9358c5161aa467c2438859caa136dec86c26fdd2efe17b92",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/once_cell/1.19.0/download"],
        strip_prefix = "once_cell-1.19.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.once_cell-1.19.0.bazel"),
    ),
    dict(
        name = "crate__paste-1.0.15",
        sha256 = "57c0d7b74b563b49d38dae00a0c37d4d6de9b432382b2892f0574ddcae73fd0a",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/paste/1.0.15/download"],
        strip_prefix = "paste-1.0.15",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.paste-1.0.15.bazel"),
    ),
    dict(
        name = "crate__proc-macro2-1.0.86",
        sha256 = "5e719e8df665df0d1c8fbfd238015744736151d4445ec0836b8e628aae103b77",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/proc-macro2/1.0.86/download"],
        strip_prefix = "proc-macro2-1.0.86",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.proc-macro2-1.0.86.bazel"),
    ),
    dict(
        name = "crate__quote-1.0.37",
        sha256 = "b5b9d34b8991d19d98081b46eacdd8eb58c6f2b201139f7c5f643cc155a633af",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/quote/1.0.37/download"],
        strip_prefix = "quote-1.0.37",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.quote-1.0.37.bazel"),
    ),
    dict(
        name = "crate__ryu-1.0.18",
        sha256 = "f3cb5ba0dc43242ce17de99c180e96db90b235b8a9fdc9543c96d2209116bd9f",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/ryu/1.0.18/download"],
        strip_prefix = "ryu-1.0.18",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.ryu-1.0.18.bazel"),
    ),
    dict(
        name = "crate__serde-1.0.210",
        sha256 = "c8e3592472072e6e22e0a54d5904d9febf8508f65fb8552499a1abc7d1078c3a",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/serde/1.0.210/download"],
        strip_prefix = "serde-1.0.210",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.serde-1.0.210.bazel"),
    ),
    dict(
        name = "crate__serde_derive-1.0.210",
        sha256 = "243902eda00fad750862fc144cea25caca5e20d615af0a81bee94ca738f1df1f",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/serde_derive/1.0.210/download"],
        strip_prefix = "serde_derive-1.0.210",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.serde_derive-1.0.210.bazel"),
    ),
    dict(
        name = "crate__serde_json-1.0.128",
        sha256 = "6ff5456707a1de34e7e37f2a6fd3d3f808c318259cbd01ab6377795054b483d8",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/serde_json/1.0.128/download"],
        strip_prefix = "serde_json-1.0.128",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.serde_json-1.0.128.bazel"),
    ),
    dict(
        name = "crate__strsim-0.10.0",
        sha256 = "73473c0e59e6d5812c5dfe2a064a6444949f089e20eec9a2e5506596494e4623",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/strsim/0.10.0/download"],
        strip_prefix = "strsim-0.10.0",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.strsim-0.10.0.bazel"),
    ),
    dict(
        name = "crate__syn-1.0.109",
        sha256 = "72b64191b275b66ffe2469e8af2c1cfe3bafa67b529ead792a6d0160888b4237",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/syn/1.0.109/download"],
        strip_prefix = "syn-1.0.109",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.syn-1.0.109.bazel"),
    ),
    dict(
        name = "crate__syn-2.0.79",
        sha256 = "89132cd0bf050864e1d38dc3bbc07a0eb8e7530af26344d3d2bbbef83499f590",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/syn/2.0.79/download"],
        strip_prefix = "syn-2.0.79",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.syn-2.0.79.bazel"),
    ),
    dict(
        name = "crate__thiserror-1.0.64",
        sha256 = "d50af8abc119fb8bb6dbabcfa89656f46f84aa0ac7688088608076ad2b459a84",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/thiserror/1.0.64/download"],
        strip_prefix = "thiserror-1.0.64",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.thiserror-1.0.64.bazel"),
    ),
    dict(
        name = "crate__thiserror-impl-1.0.64",
        sha256 = "08904e7672f5eb876eaaf87e0ce17857500934f4981c4a0ab2b4aa98baac7fc3",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/thiserror-impl/1.0.64/download"],
        strip_prefix = "thiserror-impl-1.0.64",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.thiserror-impl-1.0.64.bazel"),
    ),
    dict(
        name = "crate__unicode-ident-1.0.13",
        sha256 = "e91b56cd4cadaeb79bbf1a5645f6b4f8dc5bde8834ad5894a8db35fda9efa1fe",
        type = "tar.gz",
        urls = ["https://static.crates.io/crates/unicode-ident/1.0.13/download"],
        strip_prefix = "unicode-ident-1.0.13",
        build_file = Label("@drake//tools/workspace/crate_universe/lock/details:BUILD.unicode-ident-1.0.13.bazel"),
    ),
]
