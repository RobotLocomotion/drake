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
  
  // todo: move these to setup?
  glDisable (GL_BLEND);
  glDisable (GL_CULL_FACE);
  glEnable (GL_DEPTH_TEST);
  glEnable (GL_RESCALE_NORMAL);
//  glEnable (GL_CULL_FACE);
//  glCullFace (GL_BACK);
//  glFrontFace (GL_CCW);
//  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//  glShadeModel (GL_SMOOTH);

  glMatrixMode (GL_MODELVIEW);

  /* give the ambient light a blue tint to match the blue sky */
  float light0_amb[] = { 0.4, 0.4, .5, 1 };
  float light0_dif[] = { 1, 1, 1, 1 };
  float light0_spe[] = { .5, .5, .5, 1 };
  float light0_pos[] = { 100, 100, 100, 0 };
  glLightfv (GL_LIGHT0, GL_AMBIENT, light0_amb);
  glLightfv (GL_LIGHT0, GL_DIFFUSE, light0_dif);
  glLightfv (GL_LIGHT0, GL_SPECULAR, light0_spe);
  glLightfv (GL_LIGHT0, GL_POSITION, light0_pos);
  glEnable (GL_LIGHT0);

  glEnable (GL_LIGHTING);
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  glEnable (GL_COLOR_MATERIAL);

  Matrix<double,7,1> pose;
  
  // iterate over each model
  for (int robot=0; robot< self->model->urdf_model.size(); robot++) {

  	// iterate over each link and draw
  	for (map<string, boost::shared_ptr<urdf::Link> >::iterator l=self->model->urdf_model[robot]->links_.begin(); l!=self->model->urdf_model[robot]->links_.end(); l++) {
			if (l->second->visual) { // then at least one default visual tag exists

				int body_ind;
				if (l->second->parent_joint) {
					map<string, int>::iterator j2 = self->model->joint_map[robot].find(l->second->parent_joint->name);
					if (j2 == self->model->joint_map[robot].end()) continue;  // this shouldn't happen, but just in case...
					body_ind = j2->second;
				} else {
					map<string, int>::iterator j2 = self->model->joint_map[robot].find("floating_base");
					if (j2 == self->model->joint_map[robot].end()) continue;  // this shouldn't happen, but just in case...
					body_ind = j2->second;  // then it's attached directly to the floating base
				}

				cout << "drawing robot " << robot << " body_ind " << body_ind << ": " << self->model->bodies[body_ind].linkname << endl;

				self->model->forwardKin(body_ind,zero,2,pose);
	//      cout << l->second->name << " is at " << pose.transpose() << endl;

				double* posedata = pose.data();
				bot_quat_to_angle_axis(&posedata[3], &theta, axis);

				glPushMatrix();
				glTranslatef(pose(0),pose(1),pose(2));
				glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]);

				// todo: iterate over all visual groups (not just "default")
				map<string, boost::shared_ptr<vector<boost::shared_ptr<urdf::Visual> > > >::iterator v_grp_it = l->second->visual_groups.find("default");
				if (v_grp_it == l->second->visual_groups.end()) continue;

				vector<boost::shared_ptr<urdf::Visual> > *visuals = v_grp_it->second.get();
				for (vector<boost::shared_ptr<urdf::Visual> >::iterator viter = visuals->begin(); viter!=visuals->end(); viter++)
				{
					urdf::Visual * vptr = viter->get();
					if (!vptr) continue;

					glPushMatrix();

					// handle visual material
					if (vptr->material) {
						glColor4f(vptr->material->color.r,
								vptr->material->color.g,
								vptr->material->color.b,
								vptr->material->color.a);
	/*
						 GLfloat mat[4] = { vptr->material->color.r,
								 vptr->material->color.g,
								 vptr->material->color.b,
								 vptr->material->color.a };
						 GLfloat white[4] = {1,1,1,1};
						 glMaterialfv(GL_FRONT,GL_DIFFUSE,mat);
						 glMaterialfv(GL_FRONT,GL_AMBIENT,mat);
						 glMaterialfv(GL_FRONT,GL_SPECULAR,white); */
					}

					// todo: handle textures here?

					// handle visual origin
					glTranslatef(vptr->origin.position.x,
									vptr->origin.position.y,
									vptr->origin.position.z);

					quat[0] = vptr->origin.rotation.w;
					quat[1] = vptr->origin.rotation.x;
					quat[2] = vptr->origin.rotation.y;
					quat[3] = vptr->origin.rotation.z;
					bot_quat_to_angle_axis(quat, &theta, axis);
					glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]);

					int type = vptr->geometry->type;
					if (type == urdf::Geometry::SPHERE) {
						boost::shared_ptr<urdf::Sphere> sphere(boost::dynamic_pointer_cast<urdf::Sphere>(vptr->geometry));
						double radius = sphere->radius;
						glutSolidSphere(radius,36,36);
					} else if (type == urdf::Geometry::BOX) {
						boost::shared_ptr<urdf::Box> box(boost::dynamic_pointer_cast<urdf::Box>(vptr->geometry));
						glScalef(box->dim.x,box->dim.y,box->dim.z);
	//          glutSolidCube(1.0);
						bot_gl_draw_cube();
	//          glutSolidSphere(1,36,36);
					} else if  (type == urdf::Geometry::CYLINDER) {
						boost::shared_ptr<urdf::Cylinder> cyl(boost::dynamic_pointer_cast<urdf::Cylinder>(vptr->geometry));

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
						boost::shared_ptr<urdf::Mesh> mesh(boost::dynamic_pointer_cast<urdf::Mesh>(vptr->geometry));
						glScalef(mesh->scale.x,mesh->scale.y,mesh->scale.z);
						map<string,BotWavefrontModel*>::iterator iter = self->model->mesh_map.find(mesh->filename);
						if (iter!= self->model->mesh_map.end()) {
							bot_wavefront_model_gl_draw(iter->second);
	//        		glmDraw(iter->second->glm_model, GLM_SMOOTH);
						}
					}
					glPopMatrix();
				}
      
				glPopMatrix();
			}
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
        cout << "loading urdf: " << msg->command_data << endl;
        self->model = loadURDFfromFile(msg->command_data);
        if (self->model) {
        	MatrixXd q0 = MatrixXd::Zero(self->model->num_dof,1);
        	self->model->doKinematics(q0.data());
        }
      }
      break;

    default:
      cerr << "viewer command " << msg->command_type << " not implemented yet" << endl;
      break;
  }

  bot_viewer_request_redraw(self->viewer);
}

static void handle_lcm_robot_state(const lcm_recv_buf_t *rbuf, const char * channel, 
        const lcmt_robot_state * msg, void * user)
{
  RendererData *self = (RendererData*) user;
  
  if (!self->model) return;
  
  MatrixXd q = MatrixXd::Zero(self->model->num_dof,1);

/*
  int* robot_map = new int[msg->num_robots];
  for (int i=0; i<msg->num_robots; i++)
  {
    map<string, int>::iterator iter = self->model->robot_map.find(msg->robot_name[i]);
    if (iter==self->model->robot_map.end()) {
    	cerr << "couldn't find robot name " << msg->robot_name[i] << endl;
    	robot_map[i] = -1;
    } else {
    	robot_map[i]=iter->second;
    }
  }
*/

  for (int i=0; i<msg->num_joints; i++)
  {
//  	int robot = robot_map[msg->joint_robot[i]];
//  	if (robot<0) continue;
  	int robot = msg->joint_robot[i];

    map<string, int>::iterator iter = self->model->dof_map[robot].find(msg->joint_name[i]);
    if (iter==self->model->dof_map[robot].end())
      cerr << "couldn't find dof named " << msg->joint_name[i] << endl;
    else {
      q(iter->second) = (double) msg->joint_position[i];
//      cout << self->model->bodies[iter->second+1].jointname << " = " << q(iter->second) << ", " << iter->second << "=" << self->model->bodies[iter->second+1].dofnum << endl;
    }
  }
  
  self->model->doKinematics(q.data());
  
  /*
  // some debugging info
  cout << "q = " << q.transpose() << endl;
  const Vector4d zero(0,0,0,1);
  for (int i=0; i<=self->model->num_bodies; i++) {
    cout << "forward kin: " << self->model->bodies[i].linkname << " is at " << self->model->forwardKin(i,zero,2).transpose() << ", joint:" << self->model->bodies[i].jointname << endl;
  } 
*/
  
  bot_viewer_request_redraw(self->viewer);
//  delete[] robot_map;
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
  renderer->name = new char[strlen(name)+1];
  strcpy(renderer->name,name);
  renderer->enabled = 1;
  renderer->user = self;
  
  lcmt_viewer_command_subscribe(lcm,"DRAKE_VIEWER_COMMAND",&handle_lcm_viewer_command,self);
  lcmt_robot_state_subscribe(lcm,"DRAKE_VIEWER_STATE",&handle_lcm_robot_state,self);
  
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
