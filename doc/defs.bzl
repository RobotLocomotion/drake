# -*- python -*-

# This file contains constants that help define Drake's documentation targets
# (i.e., these should only be used for BUILD files within @drake//doc/...).

# Unless `setup/ubuntu/install_prereqs.sh --with-doc-only` has been run, most
# targets in //doc/... will fail to build, so by default we'll disable them.
#
# A developer will have to explicitly opt-in in order to build these.
DEFAULT_BINARY_TAGS = [
    "manual",
]

# Unless `setup/ubuntu/install_prereqs.sh --with-doc-only` has been run, most
# tests in //doc/... will fail to pass, so by default we'll disable them.
#
# A developer will have to explicitly opt-in in order to test these.
DEFAULT_TEST_TAGS = [
    "manual",
    # None of our documentation tools should hit the internet, but their
    # ecosystems might be doing so without us being aware.
    "block-network",
]
