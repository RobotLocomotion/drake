from drake.tools.workspace.drake_visualizer.plugin import show_image

# Activate the plugin if this script is run directly; store the results to keep
# the plugin objects in scope.
if __name__ == "__main__":
    image_viewer = show_image.init_visualizer()
