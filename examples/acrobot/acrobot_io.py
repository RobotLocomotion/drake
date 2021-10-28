import numpy as np

from pydrake.common.yaml import yaml_load, yaml_dump


def load_scenario(*, filename=None, data=None):
    """Given a scenario `filename` xor `data`  loads and
    returns the acrobot scenario from the file.
    """
    return yaml_load(filename=filename, data=data)


def save_scenario(*, scenario):
    """Given a scenario, returns a yaml-formatted str for it.
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
    return yaml_dump(scrubbed)


def load_output(*, filename=None, data=None):
    """Given an acrobot output `filename` xor `data`, loads and returns the
    np.ndarray.
    """
    x_tape_data = yaml_load(filename=filename, data=data)
    if not x_tape_data:
        raise RuntimeError("Could not load acrobot output")
    if "x_tape" not in x_tape_data:
        raise RuntimeError(f"Did not find 'x_tape' in {x_tape_data}")
    return np.array(x_tape_data["x_tape"])


def save_output(*, x_tape):
    """Given an acrobot output `x_tape`, returns a yaml-formatter str for it.
    """
    return yaml_dump({"x_tape": x_tape.tolist()})
