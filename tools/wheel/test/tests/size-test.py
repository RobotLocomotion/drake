#!/usr/bin/env python

import os
import sys

# Check that wheel size is within PyPI's per-wheel size quota.
wheel_name = os.path.basename(sys.argv[1])
wheel_size = os.path.getsize(sys.argv[1])
wheel_size_in_mib = wheel_size / float(1 << 20)
pypi_wheel_max_size = 100 << 20  # 100 MiB
print(f'{wheel_name}: {wheel_size_in_mib:.2f} MiB')

fail_message = f'Wheel is too large ({wheel_size_in_mib:.2f} MiB) for PyPI'
assert wheel_size < pypi_wheel_max_size, fail_message
