from drake.tools.workspace.drake_visualizer.plugin import (
    show_time, warn_old_script)

warn_old_script(new_module=show_time)

# Activate the plugin if this script is run directly; store the results to keep
# the plugin objects in scope.
if __name__ == "__main__":
    time_viz = show_time.init_visualizer()
