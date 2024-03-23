from pathlib import Path

import install_test_helper


def main():
    # Because repo.bzl is both troublesome and deprecated, we only perform the
    # narrowest of tests for it -- merely that it exists.
    install_dir = Path(install_test_helper.get_install_dir())
    assert (install_dir / "share/drake/repo.bzl").exists()


if __name__ == '__main__':
    main()
