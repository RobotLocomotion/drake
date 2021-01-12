import numpy as np
import yaml

from pydrake.common.yaml import yaml_load, yaml_dump


def load_scenario(*, filename=None, data=None, scenario_name=None):
    """Given a scenario `filename` xor `data`, and optionally `scenario_name`,
    loads and returns one acrobot scenario from the file.  The `scenario_name`
    may only be omitted when the file contains a single scenario.
    """
    scenarios = yaml_load(filename=filename, data=data)
    if scenario_name:
        result = scenarios[scenario_name]
    else:
        if len(scenarios) != 1:
            raise RuntimeError(
                "A scenario_name is required because the scenario file "
                "contains more than one scenario.")
        (_, result), = scenarios.items()
    return result


def save_scenario(*, scenario, scenario_name):
    """Given a scenario and its name, returns a yaml-formatted str for it.
    """
    # For a known list of scenario-specific items, convert numpy arrays into
    # lists for serialization purposes.
    scrubbed = dict(scenario)
    for key in ["controller_params", "initial_state"]:
        if isinstance(scenario[key], dict):
            for subkey in ["min", "max"]:
                scrubbed[key][subkey] = [
                    float(x) for x in scenario[key][subkey]
                ]
        else:
            scrubbed[key] = [float(x) for x in scenario[key]]
    return yaml_dump({scenario_name: scrubbed})


def load_output(*, filename=None, data=None):
    """Given an acrobot output `filename` xor `data`, loads and returns the
    np.ndarray. NOTE: We re-implement this here (rather than using
    `yaml_load`) in order to use `yaml.CLoader`, which greatly improves
    performance, but cannot be used for all the cases supported by
    yaml_load.
    """
    if sum(bool(x) for x in [data, filename]) != 1:
        raise RuntimeError("Must specify exactly one of data= and filename=")
    if data:
        x_tape_data = yaml.load(data, Loader=yaml.CLoader)
    else:
        with open(filename, "r") as data:
            x_tape_data = yaml.load(data, Loader=yaml.CLoader)
    if "x_tape" not in x_tape_data:
        raise RuntimeError(f"Did not find 'x_tape' in {x_tape_data}")
    return np.array(x_tape_data["x_tape"])


def save_output(*, x_tape):
    """Given an acrobot output `x_tape`, returns a yaml-formatter str for it.
    """
    return yaml_dump({"x_tape": x_tape.tolist()})
