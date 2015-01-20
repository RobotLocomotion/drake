#include <map>
#include <list>

#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>

#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include "URDFRigidBodyManipulator.h"
#include "lcmtypes/drake.h"
#include "drake_urdf_renderer.h"

#define UNUSED(x) (void)(x);

using namespace std;

class Geometry {
public:
  virtual ~Geometry(void) {};
  virtual void draw(void) = 0;
};

class Sphere : public Geometry {
public:
  Sphere(float r) : radius(r) {};

  double radius;
  virtual void draw(void) {
    glutSolidSphere(radius,36,36);
  }
};

class Box : public Geometry {
public:
  Box(float x, float y, float z) : dim_x(x), dim_y(y), dim_z(z) {};

  float dim_x, dim_y, dim_z;

  virtual void draw(void) {
    glScalef(dim_x,dim_y,dim_z);
    // glutSolidCube(1.0);
    bot_gl_draw_cube();
  }
};

class Cylinder : public Geometry {
public:
  Cylinder(float r, float l) : radius(r), length(l) {};

  double radius, length;

  virtual void draw(void) {
    GLUquadricObj* quadric = gluNewQuadric();
    gluQuadricDrawStyle(quadric, GLU_FILL);
    gluQuadricNormals(quadric, GLU_SMOOTH);
    gluQuadricOrientation(quadric, GLU_OUTSIDE);

    // transform from center of cylinder
    glTranslatef(0.0,0.0,-length/2.0);
    
    // first end cap
    gluDisk(quadric, 0.0, radius, 24, 1);

    // draw the cylinder
    gluCylinder(quadric,
		radius,
		radius,
		length,
		36,
		1);
    
    // second end cap
    glTranslatef(0.0,0.0,length);
    gluDisk(quadric, 0.0, radius, 24, 1);
    
    gluDeleteQuadric(quadric);
  }

};

class Mesh : public Geometry {
public:
  Mesh(string fname, int num_scale_factors, float *scale=NULL) {
    if ( scale && num_scale_factors == 3 ) {
      scale_x = scale[0];
      scale_y = scale[1];
      scale_z = scale[2];  
    }
    else if ( scale && num_scale_factors == 1 ) {
      scale_x = scale_y = scale_z = scale[0];
    }

    boost::filesystem::path mypath(fname);
    string ext = mypath.extension().native();
    boost::to_lower(ext);
	      
    if (ext.compare(".obj")==0) {
      // cout << "Loading mesh from " << fname << " (scale = " << scale << ")" << endl;
      pmesh = bot_wavefront_model_create(fname.c_str());
    } else if ( boost::filesystem::exists( mypath.replace_extension(".obj") ) ) {
      // try changing the extension to obj and loading
      //      cout << "Loading mesh from " << mypath.replace_extension(".obj").native() << endl;
      pmesh = bot_wavefront_model_create(mypath.replace_extension(".obj").native().c_str());
    }      

    if (!pmesh) {
      cerr << "Warning: Mesh " << fname << " ignored because it does not have extension .obj (nor can I find a juxtaposed file with a .obj extension)" << endl;
    }
  }
  virtual ~Mesh(void) {
    bot_wavefront_model_destroy(pmesh);
  }

  virtual void draw(void) {
    glScalef(scale_x,scale_y,scale_z);
    bot_wavefront_model_gl_draw(pmesh);
  }  

  float scale_x, scale_y, scale_z; 
  BotWavefrontModel* pmesh;
};

class Capsule : public Geometry {
public:
  Capsule(float r, float l) : radius(r), length(l) {};

  double radius, length;

  virtual void draw(void) {
    // transform to center of capsule
    glTranslatef(0.0,0.0,-length/2.0);
    glutSolidSphere(radius,36,36);
    
    GLUquadricObj* quadric = gluNewQuadric();
    gluQuadricDrawStyle(quadric, GLU_FILL);
    gluQuadricNormals(quadric, GLU_SMOOTH);
    gluQuadricOrientation(quadric, GLU_OUTSIDE);
    gluCylinder(quadric,
		radius,
		radius,
		length,
		36,
		1);
    gluDeleteQuadric(quadric);
    glTranslatef(0.0,0.0,length);
    glutSolidSphere(radius,36,36);
  }

};

class LinkGeometry {
 public:

  boost::shared_ptr<Geometry> pGeometry;
  double pos[3], theta, axis[3];
  float color[4];

  LinkGeometry(const drake_lcmt_viewer_geometry_data * geometry_data) : theta(0.0)
  {
    pos[0] = pos[1] = pos[2] = 0.0;
    axis[0] = axis[1] = axis[2] = 0.0;
    color[0] = color[1] = color[2] = .7f; color[3] = 1.0f;  // todo: pick better defaults

    switch (geometry_data->type) {
    case DRAKE_LCMT_VIEWER_GEOMETRY_DATA_SPHERE:
      {
	boost::shared_ptr<Geometry> g(boost::static_pointer_cast<Geometry>(new Sphere(geometry_data->float_data[0])));
	pGeometry = g;
      }
      break;
    case DRAKE_LCMT_VIEWER_GEOMETRY_DATA_BOX:
      {
	boost::shared_ptr<Geometry> g(boost::static_pointer_cast<Geometry>(new Box(geometry_data->float_data[0], geometry_data->float_data[1], geometry_data->float_data[2])));
	pGeometry = g;
      }
      break;
    case DRAKE_LCMT_VIEWER_GEOMETRY_DATA_CYLINDER:
      {
	boost::shared_ptr<Geometry> g(boost::static_pointer_cast<Geometry>(new Cylinder(geometry_data->float_data[0], geometry_data->float_data[1])));
	pGeometry = g;
      }
      break;
    case DRAKE_LCMT_VIEWER_GEOMETRY_DATA_MESH:
      {
	boost::shared_ptr<Geometry> g(boost::static_pointer_cast<Geometry>(new Mesh(geometry_data->string_data, geometry_data->num_float_data, geometry_data->float_data)));
	pGeometry = g;
      }
      break;
    case DRAKE_LCMT_VIEWER_GEOMETRY_DATA_CAPSULE:
      {
	boost::shared_ptr<Geometry> g(boost::static_pointer_cast<Geometry>(new Capsule(geometry_data->float_data[0], geometry_data->float_data[1])));
	pGeometry = g;
      }
      break;
    default:
      cerr << "unknown geometry type" << endl;
      break;
    }

    if (!pGeometry) {
      cerr << "failed to construct link geometry" << endl;
      return;
    }
    
    for (int i=0; i<3; i++) pos[i] = geometry_data->position[i];
    double q[4];
    for (int i=0; i<4; i++) q[i] = geometry_data->quaternion[i];
    bot_quat_to_angle_axis(q, &theta, axis);
    for (int i=0; i<4; i++) color[i] = geometry_data->color[i];
  }
  
  void draw(void) {
    glPushMatrix();
    glTranslatef(pos[0],pos[1],pos[2]);
    glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]);

    glColor4f(color[0],color[1],color[2],color[3]);
    pGeometry->draw();

    glPopMatrix();
  }
};

class Link {
public:
  double pos[3], theta, axis[3];
  list<boost::shared_ptr<LinkGeometry>> geometry;

  Link(const drake_lcmt_viewer_link_data* link_data) : theta(0.0)
  {
    pos[0] = pos[1] = pos[2] = 0.0;
    axis[0] = axis[1] = axis[2] = 0.0;
    for (int i=0; i<link_data->num_geom; i++) {
      boost::shared_ptr<LinkGeometry> g(new LinkGeometry(&(link_data->geom[i])));
      geometry.push_back(g);
    }
  }

  void draw(void) {
    glPushMatrix();
    glTranslatef(pos[0],pos[1],pos[2]);
    glRotatef(theta * 180/3.141592654, axis[0], axis[1], axis[2]);

    for (list<boost::shared_ptr<LinkGeometry>>::iterator g=geometry.begin(); g!=geometry.end(); ++g) {
      (*g)->draw();
    }

    glPopMatrix();
  }

  void update(float position[3], float quaternion[4])
  {
    for (int i=0; i<3; i++) pos[i] = position[i];
    double q[4];
    for (int i=0; i<4; i++) q[i] = quaternion[i];
    bot_quat_to_angle_axis(q, &theta, axis);
  }
};

typedef std::pair<int,string> link_index;  // robot num and link name

typedef struct _RendererData {
  BotRenderer renderer;
  BotViewer   *viewer;
  lcm_t       *lcm;
  //  map<string, BotWavefrontModel*> meshes;
  map<link_index, boost::shared_ptr<Link> > links; 
  string  movie_path;
} RendererData;

static void my_free( BotRenderer *renderer )
{
  RendererData *self = (RendererData*) renderer->user;
  
  free( self );
}

static void my_draw( BotViewer *viewer, BotRenderer *renderer )
{
  UNUSED(viewer);
  RendererData *self = (RendererData*) renderer->user;
  if (self->links.size() < 1) return;  

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

  for (map<link_index,boost::shared_ptr<Link>>::iterator l=self->links.begin(); l!=self->links.end(); ++l) {
    l->second->draw();
  }
}

static void handle_lcm_viewer_load_robot(const lcm_recv_buf_t *rbuf, const char * channel, 
        const drake_lcmt_viewer_load_robot * msg, void * user)
{
  RendererData *self = (RendererData*) user;
  self->links.clear();

  cout << "loading new robot with " << msg->num_links << " links" << endl;

  // parse new links
  for (int i=0; i<msg->num_links; i++) {
    if (msg->link[i].robot_num < 0) {
      cerr << "illegal robot_num" << endl;
      continue;
    }
    boost::shared_ptr<Link> l(new Link(&(msg->link[i])));
    self->links.insert(make_pair(make_pair(msg->link[i].robot_num,msg->link[i].name),l));
  }

  // send ack that model was successfully loaded
  drake_lcmt_viewer_command status_message;
  status_message.command_type = DRAKE_LCMT_VIEWER_COMMAND_STATUS;
  status_message.command_data = (char*) "successfully loaded robot";
  drake_lcmt_viewer_command_publish(self->lcm, "DRAKE_VIEWER_STATUS", &status_message);
  cout << "successfully loaded model" << endl;

  bot_viewer_request_redraw(self->viewer);
}

static void handle_lcm_viewer_command(const lcm_recv_buf_t *rbuf, const char * channel, 
        const drake_lcmt_viewer_command * msg, void * user)
{
  RendererData *self = (RendererData*) user;
  
  switch(msg->command_type) {
  case DRAKE_LCMT_VIEWER_COMMAND_START_RECORDING:
    {
      bot_viewer_start_recording(self->viewer);
      string movie_path(self->viewer->movie_path);
      self->movie_path = movie_path;
    }
    break;
  case DRAKE_LCMT_VIEWER_COMMAND_STOP_RECORDING:
    {
      bot_viewer_stop_recording(self->viewer);

      drake_lcmt_viewer_command status_message;
      status_message.command_type = DRAKE_LCMT_VIEWER_COMMAND_STATUS;
      status_message.command_data = const_cast<char*>(self->movie_path.c_str());
      drake_lcmt_viewer_command_publish(self->lcm, "DRAKE_VIEWER_STATUS", &status_message);
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

static void handle_lcm_viewer_draw(const lcm_recv_buf_t *rbuf, const char * channel, 
        const drake_lcmt_viewer_draw * msg, void * user)
{
  RendererData *self = (RendererData*) user;
  
  if (self->links.size()<1) return;
  
  for (int i=0; i<msg->num_links; i++) {
    map<link_index,boost::shared_ptr<Link>>::iterator iter = self->links.find(make_pair(msg->robot_num[i],msg->link_name[i]));
    if (iter == self->links.end())
      cerr << "failed to find link: " << msg->link_name[i] << endl;
    else {
      iter->second->update(msg->position[i],msg->quaternion[i]);
    }
  }

  bot_viewer_request_redraw(self->viewer);
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
  renderer->name = new char[strlen(name)+1];
  strcpy(renderer->name,name);
  renderer->enabled = 1;
  renderer->user = self;
  
  drake_lcmt_viewer_command_subscribe(lcm,"DRAKE_VIEWER_COMMAND",&handle_lcm_viewer_command,self);
  //  drake_lcmt_robot_state_subscribe(lcm,"DRAKE_VIEWER_STATE",&handle_lcm_robot_state,self);
  drake_lcmt_viewer_load_robot_subscribe(lcm,"DRAKE_VIEWER_LOAD_ROBOT",&handle_lcm_viewer_load_robot,self);
  drake_lcmt_viewer_draw_subscribe(lcm,"DRAKE_VIEWER_DRAW",&handle_lcm_viewer_draw,self);
  
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
