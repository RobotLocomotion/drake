load("//tools:install.bzl", "install")

def install_data():
    models_extensions = [
        "csv",
        "json",
        "stl",
        "obj",
        "sdf",
        "urdf",
        "xml",
        "dae",
    ]
    exclude_patterns = ["**/test/*", "**/test*"]
    prod_models_include = ["**/*.{}".format(x) for x in models_extensions]
    test_models_include = [
        p + m for p in exclude_patterns for m in models_extensions
    ]
    native.filegroup(
        name = "prod_models",
        srcs = native.glob(
            include = prod_models_include,
            exclude = exclude_patterns,
        ),
        visibility = ["//visibility:public"],
    )

    native.filegroup(
        name = "test_models",
        srcs = native.glob(
            include = test_models_include,
        ),
        visibility = ["//visibility:public"],
    )

    native.filegroup(
        name = "models",
        srcs = [":test_models", ":prod_models"],
        visibility = ["//visibility:public"],
    )

    install(
        name = "install_data",
        data = [":prod_models"],
        data_dest = "share/drake/" + native.package_name(),
        visibility = ["//visibility:public"],
    )
