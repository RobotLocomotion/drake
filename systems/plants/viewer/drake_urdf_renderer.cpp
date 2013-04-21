#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>

#include "../urdf.h"

// todo: include lcm types

#include "drake_urdf_renderer.h"

typedef struct _RendererData {
    BotRenderer renderer;
    BotViewer   *viewer;
    lcm_t       *lcm;
    RigidBodyManipulator  *model;
} RendererData;

static void my_free( BotRenderer *renderer )
{
    RendererData *self = (RendererData*) renderer->user;

    free( self->model );
    free( self );
}

static void my_draw( BotViewer *viewer, BotRenderer *renderer )
{
    RendererData *self = (RendererData*) renderer->user;

    // iterate over each link and draw
}


void 
drake_urdf_add_renderer_to_viewer(BotViewer* viewer, lcm_t* lcm, int priority)
{
    RendererData *self =
        (RendererData*) calloc(1, sizeof(RendererData));

    BotRenderer *renderer = &self->renderer;
    const char* name = "Drake URDF";
    
    self->lcm = lcm;
    self->viewer = viewer;

    renderer->draw = my_draw;
    renderer->destroy = my_free;
    renderer->name = new char[strlen(name)]; 
    strcpy(renderer->name,name);
    renderer->enabled = 1;
    renderer->user = self;

    //  todo:  subscribe to urdf listener

    //  for now, just load a hard-coded urdf:
    std::string filename("../../../examples/Atlas/urdf/atlas_minimal_contact.urdf");
    self->model = loadURDF(filename);
    
    bot_viewer_add_renderer(viewer, renderer, priority);
}

/*
 * setup_renderer:
 * Generic renderer add function for use as a dynamically loaded plugin
 */
void add_renderer_to_plugin_viewer(BotViewer *viewer, int render_priority){
  lcm_t * lcm = bot_lcm_get_global(NULL);
  drake_urdf_add_renderer_to_viewer(viewer,lcm, render_priority);
}
