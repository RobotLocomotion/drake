load("@drake//tools:install.bzl", "install")

def install_data(
    name = "models",
    prod_models_prefix = "prod_",
    test_models_prefix = "test_",
):
    """Install data

    This macro creates 3 filegroups:
    * `prod_models_prefix` + `name` (default: `prod_models`): data files in the
    current folder and its subfolders.
    * `test_models_prefix` + `name` (default: `test_models`): Only files
    contained in a subfolder named `test` or files named `*test*`.
    * `name` (default: `models`): Concatenation of the two previous filegroups.

    An install rule that copies the prod models files in the corresponding
    install directory is also created. The corresponding install directory is
    generated based on the current folder which is prepended with
    "share/drake/":
    The file `drake/drake/examples/acrobot/Acrobot_no_collision.urdf` will be
    installed in:
    {install_dir}/share/drake/drake/examples/acrobot/Acrobot_no_collision.urdf
    Test files (files contained in a `test` subfolder or named `*test*`) are
    not installed.
    """
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
        name = prod_models_prefix + name,
        srcs = native.glob(
            include = prod_models_include,
            exclude = exclude_patterns,
        ),
        visibility = ["//visibility:public"],
    )

    native.filegroup(
        name = test_models_prefix + name,
        srcs = native.glob(
            include = test_models_include,
        ),
        visibility = ["//visibility:public"],
    )

    prod_models_target = ":" + prod_models_prefix + name
    test_models_target = ":" + test_models_prefix + name
    native.filegroup(
        name = name,
        srcs = [test_models_target, prod_models_target],
        visibility = ["//visibility:public"],
    )

    install(
        name = "install_data",
        data = [prod_models_target],
        data_dest = "share/drake/" + native.package_name(),
        visibility = ["//visibility:public"],
    )
