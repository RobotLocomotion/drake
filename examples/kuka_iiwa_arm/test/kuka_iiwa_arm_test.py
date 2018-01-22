#!/usr/bin/env python

import install_test_helper


def test_run_kuka_simulation():
    install_test_helper.runAndKill(
        ['share/drake/examples/kuka_iiwa_arm/iiwa_wsg_simulation'])
    install_test_helper.runAndKill(
        ['share/drake/examples/kuka_iiwa_arm/kuka_plan_runner'])
    install_test_helper.runAndKill(
        ['share/drake/examples/kuka_iiwa_arm/kuka_simulation'])


if __name__ == '__main__':
    test_run_kuka_simulation()
