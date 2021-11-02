"""Demonstration of optimizing Spong controller parameters for an acrobot
using a Monte Carlo scenario.
"""

import argparse
from contextlib import closing
import os
import subprocess
import sys
import tempfile

import numpy as np
from scipy.optimize import fmin

from pydrake.common import FindResourceOrThrow

from drake.examples.acrobot.acrobot_io import (
    load_output, load_scenario, save_scenario)

from drake.examples.acrobot.metrics import (
    ensemble_cost, success_rate)


METRICS = {"ensemble_cost": ensemble_cost,
           "success_rate": success_rate}


def evaluate_metric_once(scenario, metric, seeds):
    """Runs one evaluation of metric by running a roll out of scenario
    with each random seed in seeds.
    """
    runner = FindResourceOrThrow(
        "drake/examples/acrobot/spong_sim_main_cc")
    env_tmpdir = os.getenv("TEST_TEMPDIR") or os.getenv("TMPDIR") or "/tmp"
    with tempfile.TemporaryDirectory(prefix="optimizer_demo",
                                     dir=env_tmpdir) as temp_dir:
        scenario_filename = os.path.join(temp_dir, "scenario.yaml")
        with open(scenario_filename, "w") as scenario_file:
            scenario_file.write(save_scenario(scenario=scenario))
        tapes = []
        for seed in seeds:
            output_filename = os.path.join(temp_dir, f"output_{seed}.yaml")
            subprocess.check_call(
                [runner,
                 "--scenario", scenario_filename,
                 "--output", output_filename,
                 "--random_seed", str(seed)])
            tapes += [load_output(filename=output_filename)]
    metric_value = metric(tapes)
    return metric_value


def optimize_controller_params(
        scenario, metric, ensemble_size, num_evaluations):
    """Runs `scipy.optimize.fmin` over the `controller_params` of scenario
    (treating the existing parameter values there as the starting point).  The
    metric is run over ensemble_size fixed random seeds.  The optimizer
    is given a budget of num_evaluations metric evaluations.

    Because each metric evaluation runs a full ensemble, the total number of
    simulations is ensemble_size * num_evaluations.
    """
    seeds = list(range(1, 1 + ensemble_size))
    metric_evaluation_count = 0

    def function_to_optimize(params):
        nonlocal metric_evaluation_count
        metric_evaluation_count += 1
        print(f"Iteration {metric_evaluation_count}: {len(seeds)} rollouts...")
        new_scenario = scenario
        new_scenario["controller_params"] = list(params)
        metric_value = evaluate_metric_once(new_scenario, metric, seeds)
        print(f"   ...{metric_evaluation_count}: metric is {metric_value}")
        return metric_value

    try:
        x0 = np.asfarray(scenario["controller_params"])
    except TypeError:
        x0 = (np.asfarray(scenario["controller_params"]["min"])
              + np.asfarray(scenario["controller_params"]["max"])) / 2

    result = fmin(func=function_to_optimize,
                  x0=x0,
                  maxfun=num_evaluations)
    return result


def main():
    parser = argparse.ArgumentParser(__doc__)
    # There is a subtlety to the default `scenario_file`: any default must be
    # a bazel `data=` dependency, but any user-specified file cannot be relied
    # on to be a bazel dependency, so we do `FindResourceOrThrow` resolution
    # only on the default and not on any user-provided argument.
    parser.add_argument(
        "--scenario_file", "-f", type=str, default=None,
        help="Scenario to run (default: example_stochastic_scenario.yaml)")
    parser.add_argument(
        "--metric", "-m", type=str, choices=METRICS.keys(),
        help="Choice of metric to optimize (default: %(default)s)",
        default="ensemble_cost")
    parser.add_argument(
        "--ensemble_size", "-e", type=int, default=10,
        help=("Size of ensemble for each cost function evaluation "
              "(default: %(default)s)"))
    parser.add_argument(
        "--num_evaluations", "-n", type=int, default=250,
        help=("Cost function call budget of the optimizer "
              "(default: %(default)s)"))
    parser.add_argument(
        "--output", "-o", type=argparse.FileType("w"), default=sys.stdout,
        help="File to write the optimized output (default: stdout)")
    args = parser.parse_args()
    with closing(args.output) as output:
        scenario_file = args.scenario_file or FindResourceOrThrow(
            "drake/examples/acrobot/test/example_stochastic_scenario.yaml")
        input_scenario = load_scenario(filename=scenario_file)
        result = optimize_controller_params(
            scenario=input_scenario,
            metric=METRICS[args.metric],
            ensemble_size=args.ensemble_size,
            num_evaluations=args.num_evaluations)
        output_scenario = input_scenario
        output_scenario["controller_params"] = result
        output.write(save_scenario(scenario=output_scenario))


if __name__ == "__main__":
    main()
