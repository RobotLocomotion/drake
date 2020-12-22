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

from drake.examples.acrobot.dev.acrobot_io import (
    load_output, load_scenario, save_scenario)

from drake.examples.acrobot.dev.metrics import (
    ensemble_cost, final_state_cost, success_rate)


METRICS = {"ensemble_cost": ensemble_cost,
           "success_rate": success_rate}


def evaluate_metric_once(scenario, metric, seeds):
    """Run one evaluation of @p metric by running a roll out of @p scenario
    with each random seed in @p seeds."""
    runner = FindResourceOrThrow(
        "drake/examples/acrobot/dev/spong_sim_main_cc")
    env_tmpdir = os.getenv("TEST_TEMPDIR") or os.getenv("TMPDIR") or "/tmp"
    with tempfile.TemporaryDirectory(prefix="optimizer_demo",
                                     dir=env_tmpdir) as temp_dir:
        scenario_filename = os.path.join(temp_dir, "scenario.yaml")

        with open(scenario_filename, "w") as scenario_file:
            scenario_file.write(
                save_scenario(scenario=scenario,
                              scenario_name=f"evaluation_scenario"))
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
    """Runs `scipy.optimize.fmin` over the `controller_params` of @p scenario
    (treating the existing parameter values there as the starting point).  The
    @p metric is run over @p ensemble_size fixed random seeds.  The optimizer
    is given a budget of @p num_evaluations metric evaluations."""
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
    parser.add_argument(
        "--scenario_file", "-f", type=str, default=None,
        help="Scenario to run (default is example_stochastic_scenario.yaml)")
    parser.add_argument(
        "--metric", "-m", type=str, choices=METRICS.keys(),
        default="ensemble_cost")
    parser.add_argument(
        "--ensemble_size", "-e", type=int, default=10,
        help="Size of ensemble for each cost function evaluation.")
    parser.add_argument(
        "--num_evaluations", "-n", type=int, default=250,
        help="Cost function call budget of the optimizer.")
    parser.add_argument(
        "--output", "-o", type=argparse.FileType("w"), default=sys.stdout,
        help="File to write the optimized output (default=stdout)")
    args = parser.parse_args()
    with closing(args.output) as output:
        scenario_file = args.scenario_file or FindResourceOrThrow(
            "drake/examples/acrobot/dev/test/example_stochastic_scenario.yaml")
        input_scenario = load_scenario(filename=scenario_file)
        result = optimize_controller_params(
            scenario=input_scenario,
            metric=METRICS[args.metric],
            ensemble_size=args.ensemble_size,
            num_evaluations=args.num_evaluations)
        output_scenario = input_scenario
        output_scenario["controller_params"] = result
        output.write(save_scenario(scenario=output_scenario,
                                   scenario_name="optimized_scenario"))


if __name__ == "__main__":
    main()
