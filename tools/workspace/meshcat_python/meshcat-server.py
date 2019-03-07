# -*- coding: utf-8 -*-
from meshcat.servers.zmqserver import main as server_main

# N.B. This is the binary for Bazel. For install, see `meshcat-server`.


def main():
    server_main()


if __name__ == '__main__':
    main()
