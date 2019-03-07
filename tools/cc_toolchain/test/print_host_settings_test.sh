#!/bin/bash
set -eux

dir=tools/cc_toolchain
exec ${dir}/print_host_settings.sh ${dir}/capture_cc.env
