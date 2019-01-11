from drake.tools.workspace.drake_visualizer.plugin import (
    show_contact, warn_old_script)

warn_old_script(new_module=show_contact)

# Activate the plugin if this script is run directly; store the results to keep
# the plugin objects in scope.
if __name__ == "__main__":
    contact_viz = show_contact.init_visualizer()
