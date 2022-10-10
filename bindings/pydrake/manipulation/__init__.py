# Bootstrap our native code.
import pydrake.common as _common

import inspect as _inspect


def ApplyDriverConfigs(*, driver_configs, sim_plant, models_from_directives,
                       lcm_buses, builder):
    """Applies many driver configurations to a model.

    A "driver configuration" helps stack Drake systems between an LCM interface
    subscriber system and the actuation input ports of a MultibodyPlant (to
    enact the driver command), as well as the output ports of a MultibodyPlant
    back to an LCM interface publisher system (to provide the driver status).

    These conceptually simulate "drivers" -- they take the role that driver
    software and control cabinets would take in real life -- but may also
    model some physical properties of the robot that are not easily reflected
    in MultibodyPlant (e.g., the WSG belt drive).

    The caller of this function is responsible for including the variant
    members' apply function, such as schunk_wsg_driver_functions.h.

    @p driver_configs The configurations to apply.
    @p sim_plant The plant containing the model.
    @p models_from_directives All of the `ModelInstanceInfo`s from previously-
    loaded directives.
    @p lcm_buses The available LCM buses to drive and/or sense from this
    driver.
    @p builder The `DiagramBuilder` into which to install this driver.
    """
    # This implements the drake/manipulation/util/apply_driver_configs.h
    # function of the same name. Due to the peculiarities of std::variant
    # and argument-dependent lookup, it's easier to re-implement rather
    # than bind using pybind11.
    models_from_directives_map = dict([
        (info.model_name, info)
        for info in models_from_directives
    ])
    for model_instance_name, driver_config in driver_configs.items():
        module = _inspect.getmodule(driver_config)
        apply_function = getattr(module, "ApplyDriverConfig")
        apply_function(
            driver_config,
            model_instance_name,
            sim_plant,
            models_from_directives_map,
            lcm_buses,
            builder)
