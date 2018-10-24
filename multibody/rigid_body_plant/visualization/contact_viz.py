from drake.tools.workspace.drake_visualizer.plugin.contact import (
    init_visualizer)

# Activate the plugin if this script is run directly; store the results to keep
# the plugin objects in scope.
if __name__ == "__main__":
    contact_viz = init_visualizer()
