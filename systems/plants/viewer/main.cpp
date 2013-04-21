#include <string.h>
#include <stdlib.h>
#include <gtk/gtk.h>
#include <iostream>

#include <bot_vis/bot_vis.h>
#include <lcm/lcm.h>
#include <bot_core/bot_core.h>

//imported renderers
#include <bot_lcmgl_render/lcmgl_bot_renderer.h>
#include "drake_urdf_renderer.h"

using namespace std;

int main(int argc, char *argv[])
{
  setlinebuf(stdout);
  string viewer_title = "Drake Viewer";
  
  gtk_init(&argc, &argv);
  glutInit(&argc, argv);
  g_thread_init(NULL);  // deprecated since 2.32, but leave in for backwards compatibility

  lcm_t *lcm = lcm_create("");
  
  bot_glib_mainloop_attach_lcm(lcm);
  BotViewer* viewer = bot_viewer_new(viewer_title.c_str());

  //die cleanly for control-c etc :-)
  bot_gtk_quit_on_interrupt();

  // core renderers
  bot_viewer_add_stock_renderer(viewer, BOT_VIEWER_STOCK_RENDERER_GRID, 1);
  bot_lcmgl_add_renderer_to_viewer(viewer, lcm, 1);
  drake_urdf_add_renderer_to_viewer(viewer, lcm, 1);
          
  // load the renderer params from the config file.
  const char *fname = ".viewer-prefs"; //g_build_filename(g_get_user_config_dir(), vis_config_file.c_str() , NULL);
  bot_viewer_load_preferences(viewer, fname);

  gtk_main();

  //save the renderer params to the config file.
  bot_viewer_save_preferences(viewer, fname);

  bot_viewer_unref(viewer);
}
