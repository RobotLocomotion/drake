from drake.tools.workspace.drake_visualizer.plugin import (
    show_frame, warn_old_script)

warn_old_script(new_module=show_frame)

# Activate the plugin if this script is run directly; store the results to keep
# the plugin objects in scope.
if __name__ == "__main__":
    frame_viz = show_frame.init_visualizer()
