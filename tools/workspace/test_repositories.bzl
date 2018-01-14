load("@drake//tools/external_data/test:external_data_workspace_test.bzl", "external_data_test_repositories")  # noqa

def add_test_repositories(workspace_dir):
    external_data_test_repositories(workspace_dir)
