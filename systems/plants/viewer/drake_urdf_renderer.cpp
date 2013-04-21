#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>

#include <boost/shared_ptr.hpp>

#include <urdf.h>

#include <lcmt_viewer_command.h>
#include <lcmt_robot_state.h>

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
  if (!self->model) return;  
  
  const Vector4d zero(0,0,0,1);
  double theta, axis[3], quat[4];
  
/*  
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_BLEND);
  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
  glEnable (GL_RESCALE_NORMAL);
*/
  
  // iterate over each link and draw
  for (std::map<std::string, boost::shared_ptr<urdf::Link> >::iterator l=self->model->urdf_model->links_.begin(); l!=self->model->urdf_model->links_.end(); l++) {
    if (l->second->visual) { // then at least one default visual tag exists
      
      int body_ind;
      if (l->second->parent_joint) {
        std::map<std::string, int>::iterator j2 = self->model->joint_map.find(l->second->parent_joint->name);
        if (j2 == self->model->joint_map.end()) continue;  // this shouldn't happen, but just in case...
        body_ind = j2->second;
      } else {
        body_ind = 5;  // then it's attached directly to the floating base
      }
      
      MatrixXd pose = self->model->forwardKin(body_ind,zero,2);
      
      double* posedata = pose.data();
      bot_quat_to_angle_axis(&posedata[3], &theta, axis);

      glPushMatrix();
      glTranslatef(pose(0),pose(1),pose(2));
      glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]); 
      
      map<string, boost::shared_ptr<vector<boost::shared_ptr<urdf::Visual> > > >::iterator v_grp_it = l->second->visual_groups.find("default");
      vector<boost::shared_ptr<urdf::Visual> > visuals = (*v_grp_it->second);
      for (size_t iv = 0;iv < visuals.size();iv++)
      {
        glPushMatrix();
        
        // handle visual material 
        glColor4f(visuals[iv]->material->color.r,
                visuals[iv]->material->color.g,
                visuals[iv]->material->color.b,
                visuals[iv]->material->color.a);

        // todo: handle textures here?
        
        // handle visual origin
        glTranslatef(visuals[iv]->origin.position.x, 
                visuals[iv]->origin.position.y,
                visuals[iv]->origin.position.z);

        quat[0] = visuals[iv]->origin.rotation.w;
        quat[1] = visuals[iv]->origin.rotation.x;
        quat[2] = visuals[iv]->origin.rotation.y;
        quat[3] = visuals[iv]->origin.rotation.z;
        bot_quat_to_angle_axis(quat, &theta, axis);
        glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]);
        
        int type = visuals[iv]->geometry->type;
        if (type == urdf::Geometry::SPHERE) {
          boost::shared_ptr<urdf::Sphere> sphere(boost::dynamic_pointer_cast<urdf::Sphere>(visuals[iv]->geometry));
          double radius = sphere->radius;
          glutSolidSphere(radius,36,36);
        } else if (type == urdf::Geometry::BOX) {
          boost::shared_ptr<urdf::Box> box(boost::dynamic_pointer_cast<urdf::Box>(visuals[iv]->geometry));
          glScalef(box->dim.x,box->dim.y,box->dim.z);
          glutSolidCube(1.0);
        } else if  (type == urdf::Geometry::CYLINDER) {
          boost::shared_ptr<urdf::Cylinder> cyl(boost::dynamic_pointer_cast<urdf::Cylinder>(visuals[iv]->geometry));
          
          // transform to center of cylinder
          glTranslatef(0.0,0.0,-cyl->length/2.0);
          
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
        glPopMatrix();
      }
      
      glPopMatrix();
    }
  }
}


static void handle_lcm_viewer_command(const lcm_recv_buf_t *rbuf, const char * channel, 
        const lcmt_viewer_command * msg, void * user)
{
  RendererData *self = (RendererData*) user;
  
  switch(msg->command_type) {
    case 1: //msg->LOAD_URDF:
      {
        if (self->model) delete self->model;
        self->model = loadURDFfromFile(msg->command_data);
        MatrixXd q0 = MatrixXd::Zero(self->model->NB,1);
        self->model->doKinematics(q0.data());
      }
      break;

    default:
      cerr << "viewer command " << msg->command_type << " not implemented yet" << endl;
      break;
  }
}

static void handle_lcm_robot_state(const lcm_recv_buf_t *rbuf, const char * channel, 
        const lcmt_robot_state * msg, void * user)
{
  RendererData *self = (RendererData*) user;
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
  self->model = NULL;
  
  renderer->draw = my_draw;
  renderer->destroy = my_free;
  renderer->name = new char[strlen(name)];
  strcpy(renderer->name,name);
  renderer->enabled = 1;
  renderer->user = self;
  
  lcmt_viewer_command_subscribe(lcm,"DRAKE_VIEWER_COMMAND",&handle_lcm_viewer_command,self);
  lcmt_robot_state_subscribe(lcm,"DRAKE_VIEWER_STATE",&handle_lcm_robot_state,self);
  
  //  for now, just load a hard-coded urdf:
//  self->model = loadURDFfromFile("../../../examples/Atlas/urdf/atlas_minimal_contact.urdf");
//  self->model = loadURDFfromFile("../test/FallingBrick.urdf");
//  self->model = loadURDFfromFile("../../../examples/Acrobot/Acrobot.urdf");
//  MatrixXd q0 = MatrixXd::Zero(self->model->NB,1);
//  self->model->doKinematics(q0.data());
  
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
