#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>

#include <boost/shared_ptr.hpp>

#include <urdf.h>

// todo: include lcm types

#include "drake_urdf_renderer.h"

using namespace std;

typedef struct _RendererData {
  BotRenderer renderer;
  BotViewer   *viewer;
  lcm_t       *lcm;
  URDFRigidBodyManipulator  *model;
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
  const Vector4d zero(0,0,0,1);
  double theta, axis[3];
  
  // iterate over each link and draw
  for (std::map<std::string, boost::shared_ptr<urdf::Link> >::iterator l=self->model->urdf_model->links_.begin(); l!=self->model->urdf_model->links_.end(); l++) {
    if (l->second->visual) { // then at least one default visual tag exists
      
      std::map<std::string, int>::iterator j2 = self->model->joint_map.find(l->second->parent_joint->name);
      if (j2 == self->model->joint_map.end()) continue;  // this shouldn't happen, but just in case...
      
      MatrixXd pose = self->model->forwardKin(j2->second,zero,2);
      double* posedata = pose.data();
      bot_quat_to_angle_axis(&posedata[3], &theta, axis);

      glPushMatrix();
      glTranslatef(pose(0),pose(1),pose(2));
      glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]); 
      
      map<string, boost::shared_ptr<vector<boost::shared_ptr<urdf::Visual> > > >::iterator v_grp_it = l->second->visual_groups.find("default");
      vector<boost::shared_ptr<urdf::Visual> > visuals = (*v_grp_it->second);
      for (size_t iv = 0;iv < visuals.size();iv++)
      {
        int type = visuals[iv]->geometry->type;
        if (type == urdf::Geometry::SPHERE) {
          boost::shared_ptr<urdf::Sphere> sphere(boost::dynamic_pointer_cast<urdf::Sphere>(visuals[iv]->geometry));
          double radius = sphere->radius;
          glutSolidSphere(radius,36,36);
        } else if (type == urdf::Geometry::BOX) {
          boost::shared_ptr<urdf::Box> box(boost::dynamic_pointer_cast<urdf::Box>(visuals[iv]->geometry));
          glPushMatrix();
          glScalef(box->dim.x,box->dim.y,box->dim.z);
          glutSolidCube(1.0);
          glPopMatrix();
        } else if  (type == urdf::Geometry::CYLINDER) {
          boost::shared_ptr<urdf::Cylinder> cyl(boost::dynamic_pointer_cast<urdf::Cylinder>(visuals[iv]->geometry));
          // todo: transform to center of cylinder
          GLUquadricObj* quadric = gluNewQuadric();
          gluQuadricDrawStyle(quadric, GLU_FILL);
          gluQuadricNormals(quadric, GLU_SMOOTH);
          gluQuadricOrientation(quadric, GLU_OUTSIDE);
          gluCylinder(quadric,
                  cyl->radius,
                  cyl->radius,
                  (double) cyl->length,
                  36,
                  1);
          gluDeleteQuadric(quadric);
        } else if (type == urdf::Geometry::MESH) {
          // not implemented yet.
          //glCallList (wavefrontmodel.displaylist);
        } 
      }
      
      glPopMatrix();
    }
  }
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
//  self->model = loadURDFfromFile("../../../examples/Atlas/urdf/atlas_minimal_contact.urdf");
  self->model = loadURDFfromFile("../test/FallingBrick.urdf");
  
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
