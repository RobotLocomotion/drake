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

    This function assumes that the ApplyDriverConfig functions associated with
    the values in the driver_configs map are defined in the same module as the
    value itself.

    Args:
        driver_configs: The configurations to apply, mapping a ``str`` model
          instance name to some kind of a DriverConfig object (e.g., a
          ``pydrake.manipulation.util.ZeroForceDriver``).
        sim_plant: The plant containing the model.
        models_from_directives: The list of ``ModelInstanceInfo`` from
          previously-loaded directives (e.g., from ``ProcessModelDirectives``).
        lcm_buses: The available LCM buses to drive and/or sense from this
          driver.
        builder: The ``DiagramBuilder`` into which to install this driver.
    """
    # This implements the drake/manipulation/util/apply_driver_configs.h
    # function of the same name. Due to the peculiarities of std::variant
    # and argument-dependent lookup, it's easier to re-implement it rather
    # than bind it via pybind11.
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
