#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>

#include <boost/shared_ptr.hpp>

#include "URDFRigidBodyManipulator.h"

#include "lcmtypes/drake.h"

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

  self->model->draw();
}


static void handle_lcm_viewer_command(const lcm_recv_buf_t *rbuf, const char * channel, 
        const drake_lcmt_viewer_command * msg, void * user)
{
  RendererData *self = (RendererData*) user;
  
  switch(msg->command_type) {
    case DRAKE_LCMT_VIEWER_COMMAND_LOAD_URDF:
      {
        if (self->model) delete self->model;
        cout << "loading urdf: " << msg->command_data << endl;
        self->model = loadURDFfromFile(msg->command_data);
        if (self->model) {
        	MatrixXd q0 = MatrixXd::Zero(self->model->num_dof,1);
        	self->model->doKinematics(q0.data());

        	// send ack that model was successfully loaded
        	drake_lcmt_viewer_command status_message;
          status_message.command_type = DRAKE_LCMT_VIEWER_COMMAND_STATUS;
          status_message.command_data = msg->command_data;
          drake_lcmt_viewer_command_publish(self->lcm, "DRAKE_VIEWER_STATUS", &status_message);
        } else {
        	cout << "failed to load urdf." << endl;
        }
      }
      break;

    case DRAKE_LCMT_VIEWER_COMMAND_LOAD_TERRAIN:
      {
      	cerr << "Terrain loading from file not implemented yet" << endl;
      }
    	break;

    case DRAKE_LCMT_VIEWER_COMMAND_SET_TERRAIN_TRANSFORM:
			{
      	cerr << "Setting the terrain transform is not implemented yet" << endl;
			}
    	break;

    default:
      cerr << "viewer command " << msg->command_type << " not implemented yet" << endl;
      break;
  }

  bot_viewer_request_redraw(self->viewer);
}

static void handle_lcm_robot_state(const lcm_recv_buf_t *rbuf, const char * channel, 
        const drake_lcmt_robot_state * msg, void * user)
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
    //DEBUG
    //for (auto p : self->model->dof_map[robot]) {
      //cout << "drake_urdf_renderer: p.first = " << p.first << endl;
    //}
    //END_DEBUG
    if (iter==self->model->dof_map[robot].end()) {
      cerr << "couldn't find dof named " << msg->joint_name[i] << endl;
    } else {
      q(iter->second) = (double) msg->joint_position[i];
      //DEBUG
      //cout << endl;
      //cout << "drake_urdf_renderer: msg->joint_name[i] = " << msg->joint_name[i] << endl;
      //cout << "drake_urdf_renderer: iter->first = " << iter->first << endl;
      //cout << "drake_urdf_renderer: iter->second = " << iter->second << endl;
      //cout << endl;
      //END_DEBUG
//      cout << self->model->bodies[iter->second+1].jointname << " = " << q(iter->second) << ", " << iter->second << "=" << self->model->bodies[iter->second+1].dofnum << endl;
    }
    //DEBUG
    //cout << "drake_urdf_renderer: q = " << endl;
    //cout << q << endl;
    //END_DEBUG
  }
  
  self->model->doKinematics(q.data());
  
/*
  // some debugging info
  cout << "q = " << q.transpose() << endl;
  const Vector4d zero(0,0,0,1);
  Vector3d pos;
  for (int i=0; i<self->model->num_bodies; i++) {
    self->model->forwardKin(i,zero,0,pos);
    cout << "forward kin: " << self->model->bodies[i].linkname << " is at " << pos.transpose() << ", joint:" << self->model->bodies[i].jointname << endl;
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
  
  drake_lcmt_viewer_command_subscribe(lcm,"DRAKE_VIEWER_COMMAND",&handle_lcm_viewer_command,self);
  drake_lcmt_robot_state_subscribe(lcm,"DRAKE_VIEWER_STATE",&handle_lcm_robot_state,self);
  
  bot_viewer_add_renderer(viewer, renderer, priority);

  drake_lcmt_viewer_command status_message;
  status_message.command_type = DRAKE_LCMT_VIEWER_COMMAND_STATUS;
  status_message.command_data = (char*) "loaded";

  drake_lcmt_viewer_command_publish(lcm, "DRAKE_VIEWER_STATUS", &status_message);
}

/*
 * setup_renderer:
 * Generic renderer add function for use as a dynamically loaded plugin
 */
void add_renderer_to_plugin_viewer(BotViewer *viewer, int render_priority){
  lcm_t * lcm = bot_lcm_get_global(NULL);
  drake_urdf_add_renderer_to_viewer(viewer,lcm, render_priority);
}
