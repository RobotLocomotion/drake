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

#ifndef GLIB_VERSION_2_32
  g_thread_init(NULL);  // deprecated since 2.32, but leave in for backwards compatibility
#endif

  lcm_t *lcm = lcm_create("");
  
  bot_glib_mainloop_attach_lcm(lcm);
  BotViewer* viewer = bot_viewer_new(viewer_title.c_str());

  //die cleanly for control-c etc :-)
  bot_gtk_quit_on_interrupt();
  
  // hide left control box (unfortunately, assumes exact gui structure used in viewer.c)
  GtkWidget* pwidget = viewer->controls_box_left;
  pwidget = gtk_widget_get_parent(pwidget);  // controls_view1
  pwidget = gtk_widget_get_parent(pwidget);  // controls_scroll1
  pwidget = gtk_widget_get_parent(pwidget);  // controls_align1
  pwidget = gtk_widget_get_parent(pwidget);  // hpaned_main
  gtk_paned_set_position(GTK_PANED(pwidget), 0);
  
  // set initial view
  const double eye[3] = {0.0,-4.0,2.0}, lookat[3] = {0.0,0.0,0.0}, up[3] = {0.0,0.0,1.0};
  viewer->view_handler->set_look_at(viewer->view_handler, eye, lookat, up); 

  // core renderers
  drake_urdf_add_renderer_to_viewer(viewer, lcm, 1);
  bot_viewer_add_stock_renderer(viewer, BOT_VIEWER_STOCK_RENDERER_GRID, 1);
  bot_lcmgl_add_renderer_to_viewer(viewer, lcm, 1);
          
  // load the renderer params from the config file.
  const char *fname = ".viewer-prefs"; //g_build_filename(g_get_user_config_dir(), vis_config_file.c_str() , NULL);
  bot_viewer_load_preferences(viewer, fname);

  gtk_main();

  //save the renderer params to the config file.
  bot_viewer_save_preferences(viewer, fname);

  bot_viewer_unref(viewer);
}
