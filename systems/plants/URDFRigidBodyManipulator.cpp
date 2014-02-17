
#include <stdio.h>
#include <stdarg.h>
#include <fstream>
#include <sstream>
#include <map>

#include "URDFRigidBodyManipulator.h"
#include "urdf_interface/model.h"

#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

using namespace std;

#ifdef BOT_VIS_SUPPORT
#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>

class URDFRigidBodyManipulatorWBotVis : public URDFRigidBodyManipulator
{
public:
  URDFRigidBodyManipulatorWBotVis(void) {};
  virtual ~URDFRigidBodyManipulatorWBotVis(void)
  {
    for (map<string, BotWavefrontModel*>::iterator iter = mesh_map.begin(); iter!=mesh_map.end(); iter++)
      bot_wavefront_model_destroy(iter->second);
  }

  map<string, BotWavefrontModel*> mesh_map;

  virtual bool addURDF(boost::shared_ptr<urdf::ModelInterface> _urdf_model, std::map<std::string, int> jointname_to_jointnum, std::map<std::string,int> dofname_to_dofnum, const std::string & root_dir = ".")
  {
    if (!URDFRigidBodyManipulator::addURDF(_urdf_model, jointname_to_jointnum, dofname_to_dofnum, root_dir))
      return false;

    for (map<string, boost::shared_ptr<urdf::Link> >::iterator l=_urdf_model->links_.begin(); l!=_urdf_model->links_.end(); l++) {
      // load geometry
      if (l->second->visual) { // then at least one default visual tag exists
      	// todo: iterate over all visual groups (not just "default")
      	map<string, boost::shared_ptr<vector<boost::shared_ptr<urdf::Visual> > > >::iterator v_grp_it = l->second->visual_groups.find("default");
      	for (size_t iv = 0;iv < v_grp_it->second->size();iv++)
      	{
      		vector<boost::shared_ptr<urdf::Visual> > visuals = (*v_grp_it->second);
      		if (visuals[iv]->geometry->type == urdf::Geometry::MESH) {
      			boost::shared_ptr<urdf::Mesh> mesh(boost::dynamic_pointer_cast<urdf::Mesh>(visuals[iv]->geometry));
	      
      			map<string,BotWavefrontModel*>::iterator iter = mesh_map.find(mesh->filename);
      			if (iter!=mesh_map.end())  // then it's already in the map... no need to load it again
      				continue;
	      
      			string fname = mesh->filename;
      			bool has_package = boost::find_first(mesh->filename,"package://");
      			if (has_package) {
      				cout << "replacing " << fname;
							boost::replace_first(fname,"package://","");
							string package = fname.substr(0,fname.find_first_of("/"));
							boost::replace_first(fname,package,rospack(package));
							cout << " with " << fname << endl;
      			} else {
      				fname = root_dir + "/" + mesh->filename;
      			}
      			boost::filesystem::path mypath(fname);
	      
      			if (!boost::filesystem::exists(fname)) {
      				cerr << "cannot find mesh file: " << fname;
      				if (has_package)
      					cerr << " (note: original mesh string had a package:// in it, and I haven't really implemented rospack yet)";
      				cerr << endl;
      				continue;
      			}
	      
      			string ext = mypath.extension().native();
      			boost::to_lower(ext);
	      
      			if (ext.compare(".obj")==0) {
		//            cout << "Loading mesh from " << fname << endl;
      				BotWavefrontModel* wavefront_model = bot_wavefront_model_create(fname.c_str());
      				if (!wavefront_model) {
      					cerr << "Error loading mesh: " << fname << endl;
      				} else {
      					mesh_map.insert(make_pair(mesh->filename, wavefront_model));
      				}
      			} else {
      				// try changing the extension to dae and loading
      				if ( boost::filesystem::exists( mypath.replace_extension(".obj") ) ) {
      					//             cout << "Loading mesh from " << mypath.replace_extension(".obj").native() << endl;
      					BotWavefrontModel* wavefront_model = bot_wavefront_model_create(mypath.replace_extension(".obj").native().c_str());
      					if (!wavefront_model) {
      						cerr << "Error loading mesh: " << fname << endl;
      					} else {
      						mesh_map.insert(make_pair(mesh->filename, wavefront_model));
      					}
      				} else {
      					cerr << "Warning: Mesh " << fname << " ignored because it does not have extension .obj (nor can I find a juxtaposed file with a .obj extension)" << endl;
      				}
      			}
      		}
      	}
      }
    }
    return true;
  }


  virtual void draw(void)
  {
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
    for (int robot=0; robot< urdf_model.size(); robot++) {

      // iterate over each link and draw
      for (map<string, boost::shared_ptr<urdf::Link> >::iterator l=urdf_model[robot]->links_.begin(); l!=urdf_model[robot]->links_.end(); l++) {
	if (l->second->visual) { // then at least one default visual tag exists

	  int body_ind;
	  if (l->second->parent_joint) {
	    map<string, int>::const_iterator j2 = findWithSuffix(joint_map[robot],l->second->parent_joint->name);
	    if (j2 == joint_map[robot].end()) continue;  // this shouldn't happen, but just in case...
	    body_ind = j2->second;
	  } else {
	    map<string, int>::const_iterator j2 = findWithSuffix(joint_map[robot],"base");
	    if (j2 == joint_map[robot].end()) continue;  // this shouldn't happen, but just in case...
	    body_ind = j2->second;  // then it's attached directly to the floating base
	  }

    //DEBUG
        //cout << "drawing robot " << robot << " body_ind " << body_ind << ": " << bodies[body_ind].linkname << endl;
    //END_DEBUG

	  forwardKin(body_ind,zero,2,pose);
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
		map<string,BotWavefrontModel*>::iterator iter = mesh_map.find(mesh->filename);
		if (iter!= mesh_map.end()) {
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

};

#endif


/*
 *   Works like m.find(str), except that a matching key can contain any number
 *   of digits before any underscores in `str` or at the end. Thus,
 *
 *      str="base"  matches  "base1"  or  "base2" etc.
 *      str="base_x" matches "base1_x" or "base2_x"  or "base_x1" etc.
 */
map<string,int>::const_iterator findWithSuffix(const map<string,int>& m, const string& str)
{
   
  auto first = m.begin();
  auto last = m.end();

  // We're building a regex from the given input string, so we must use a
  // quoting escape in case the input string contains regex special characters.
  // Quoting escape sequences begin with \Q and end with \E.
  std::string quotedStr = "\\Q" + str;

  // Now we want to insert a regex expression to match any number of digits
  // preceding an underscore or the end of the string ($).  So, for example,
  // we will replace "_" with "\d*_".  Since we have prefixed the string with
  // a quoating escape, we must terminate the quoting escape before inserting
  // the \d*, then continue the quoting escape sequence.  So, our final
  // replacement looks like:   _  ==>  \E\d*\Q_

  boost::regex delim("(_|$)");
  /*
   *So many '\' characters! Each std::string initialization eats one from each
   *set.
   */
  boost::regex re(boost::regex_replace(quotedStr, delim, "\\\\E\\\\d*\\\\Q\\1"));

  auto flexibleNameMatch = 
    [&](pair<string,int> p)->bool{return boost::regex_match(p.first,re);};
  //DEBUG
  //cout << "findWithSuffix: str = "<< str << endl;
  //cout << "findWithSuffix: re.str() = "<< re.str() << endl;
  //if (find_if(first,last,f)==last) {
    //for (auto p : m){
      //cout << "findWithSuffix: p.first = "<< p.first << endl;
      //cout << "f(p) = " << f(p) << endl;
    //}
  //}
  //END_DEBUG
  return find_if(first,last,flexibleNameMatch);
}


// Defines a method to load a model directly from a URDF file

void mexErrMsgIdandTxt(const char *errorid, const char *errormsg, ...)
{
  va_list vl;
  va_start(vl, errormsg);
  vprintf(errormsg, vl);
  va_end(vl);  
  printf("\n");
  exit(1);
}

string exec(string cmd)
{
	// from http://stackoverflow.com/questions/478898/how-to-execute-a-command-and-get-output-of-command-within-c
	// note: replace popen and pclose with _popen and _pclose for Windows.
	FILE* pipe = popen(cmd.c_str(), "r");
	if (!pipe) return "ERROR";
	char buffer[128];
	string result = "";
	while(!feof(pipe)) {
		if(fgets(buffer, 128, pipe) != NULL)
			result += buffer;
    }
	pclose(pipe);
	return result;
}

void searchenvvar(map<string,string> &package_map, string envvar)
{
	char* cstrpath = getenv(envvar.c_str());
	if (!cstrpath) return;

	string path(cstrpath), token, t;
	istringstream iss(path);

	while (getline(iss,token,':')) {
		istringstream p(exec("find -L "+token+" -iname manifest.xml"));
	  while (getline(p,t)) {
	  	boost::filesystem::path mypath(t);
	  	mypath = mypath.parent_path();
//	  	cout << "found package: " << mypath.filename().native() << " in " << mypath.native() << endl;
	  	package_map.insert(make_pair(mypath.filename().native(),mypath.native()));
	  }
	}
}

string rospack(string package)
{
	// my own quick and dirty implementation of the rospack algorithm (based on my matlab version in rospack.m)
	static map<string,string> package_map;

	if (package_map.empty()) {
		searchenvvar(package_map,"ROS_ROOT");
		searchenvvar(package_map,"ROS_PACKAGE_PATH");
	}

	map<string,string>::iterator iter = package_map.find(package);
	if (iter != package_map.end())
		return iter->second;

	cerr << "Couldn't find package " << package << " in ROS_ROOT or ROS_PACKAGE_PATH" << endl;
	return "";
}

void poseToTransform(const urdf::Pose& pose, Matrix4d& T)
{
  double x=pose.position.x,
          y=pose.position.y,
          z=pose.position.z;
  double qx,qy,qz,qw;
  pose.rotation.getQuaternion(qx,qy,qz,qw);
  T <<    qw*qw + qx*qx - qy*qy - qz*qz, 2*qx*qy - 2*qw*qz, 2*qx*qz + 2*qw*qy, x,
          2*qx*qy + 2*qw*qz,  qw*qw + qy*qy - qx*qx - qz*qz, 2*qy*qz - 2*qw*qx, y,
          2*qx*qz - 2*qw*qy, 2*qy*qz + 2*qw*qx, qw*qw + qz*qz - qx*qx - qy*qy, z,
          0, 0, 0, 1;
}

URDFRigidBodyManipulator::URDFRigidBodyManipulator(void)
: 
  RigidBodyManipulator(0,0,1)
{
	bodies[0].linkname = "world";
	bodies[0].parent = -1;
}

void setJointLimits(boost::shared_ptr<urdf::ModelInterface> urdf_model, boost::shared_ptr<urdf::Joint> j, const map<string, int> &dofname_to_dofnum, double* joint_limit_min, double* joint_limit_max);

bool URDFRigidBodyManipulator::addURDF(boost::shared_ptr<urdf::ModelInterface> _urdf_model, map<string, int> jointname_to_jointnum, map<string,int> dofname_to_dofnum, const string & root_dir)
{
  robot_map.insert(make_pair(_urdf_model->getName(),(int)robot_name.size()));
  robot_name.push_back(_urdf_model->getName());
  resize(num_dof + (int)dofname_to_dofnum.size(),-1,num_bodies + (int)jointname_to_jointnum.size());
  for (map<string, boost::shared_ptr<urdf::Joint> >::iterator j=_urdf_model->joints_.begin(); j!=_urdf_model->joints_.end(); j++)
  {
    setJointLimits(_urdf_model,j->second,dofname_to_dofnum,this->joint_limit_min,this->joint_limit_max);
  }
  this->joint_limit_min[dofname_to_dofnum.at("base_x")] = -1.0/0.0;
  this->joint_limit_max[dofname_to_dofnum.at("base_x")] = 1.0/0.0;
  this->joint_limit_min[dofname_to_dofnum.at("base_y")] = -1.0/0.0;
  this->joint_limit_max[dofname_to_dofnum.at("base_y")] = 1.0/0.0;
  this->joint_limit_min[dofname_to_dofnum.at("base_z")] = -1.0/0.0;
  this->joint_limit_max[dofname_to_dofnum.at("base_z")] = 1.0/0.0;
  this->joint_limit_min[dofname_to_dofnum.at("base_roll")] = -1.0/0.0;
  this->joint_limit_max[dofname_to_dofnum.at("base_roll")] = 1.0/0.0;
  this->joint_limit_min[dofname_to_dofnum.at("base_pitch")] = -1.0/0.0;
  this->joint_limit_max[dofname_to_dofnum.at("base_pitch")] = 1.0/0.0;
  this->joint_limit_min[dofname_to_dofnum.at("base_yaw")] = -1.0/0.0;
  this->joint_limit_max[dofname_to_dofnum.at("base_yaw")] = 1.0/0.0;
  joint_map.push_back(jointname_to_jointnum);
  dof_map.push_back(dofname_to_dofnum);
  urdf_model.push_back(_urdf_model);
  
  for (map<string, boost::shared_ptr<urdf::Link> >::iterator l=_urdf_model->links_.begin(); l!=_urdf_model->links_.end(); l++) {
    int index, _dofnum;
    if (l->second->parent_joint) {
    	boost::shared_ptr<urdf::Joint> j = l->second->parent_joint;
    	map<string, int>::const_iterator jn=findWithSuffix(jointname_to_jointnum,j->name);
    	if (jn == jointname_to_jointnum.end()) ROS_ERROR("can't find joint %s.  this shouldn't happen", j->name.c_str());
    	index = jn->second;
      map<string, int>::const_iterator dn=findWithSuffix(dofname_to_dofnum,j->name);
      if (dn == dofname_to_dofnum.end()) ROS_ERROR("can't find joint %s.  this shouldn't happen", j->name.c_str());
      _dofnum = dn->second;

    	bodies[index].linkname = l->first;
    	bodies[index].jointname = j->name;
        if(l->second->inertial == nullptr)
        {
          bodies[index].mass = 0.0;
        }
        else
        {
          bodies[index].mass = l->second->inertial->mass;
        }
//    	cout << "body[" << index << "] linkname: " << bodies[index].linkname << ", jointname: " << bodies[index].jointname << endl;


    	{ // set up parent
    		map<string, boost::shared_ptr<urdf::Link> >::iterator pl=_urdf_model->links_.find(j->parent_link_name);
    		if (pl == _urdf_model->links_.end()) ROS_ERROR("can't find link %s.  this shouldn't happen", j->parent_link_name.c_str());

    		if (pl->second->parent_joint) {
    			boost::shared_ptr<urdf::Joint> pj = pl->second->parent_joint;
    			map<string, int>::iterator pjn=jointname_to_jointnum.find(pj->name);
    			if (pjn == jointname_to_jointnum.end()) ROS_ERROR("can't find joint %s.  this shouldn't happen", pj->name.c_str());

    			bodies[index].parent = pjn->second;
    			parent[index-1] = pjn->second-1;
    		} else { // the parent body is the floating base
            	string jointname="base";
                map<string, int>::iterator jn=jointname_to_jointnum.find(jointname);
                if (jn == jointname_to_jointnum.end()) ROS_ERROR("can't find joint %s.  this shouldn't happen",jointname.c_str());
                bodies[index].parent = jn->second;
    		}
    	}

    	bodies[index].dofnum = _dofnum;
    	dofnum[index-1] = _dofnum;

    	// set pitch and floating
    	switch (j->type) {
    	case urdf::Joint::PRISMATIC:
    		pitch[index-1] = INF;
    		bodies[index].pitch = INF;
    		bodies[index].floating=0;
    		break;
      default:  // continuous, rotary, fixed, ...
      	pitch[index-1] = 0.0;
      	bodies[index].pitch = 0.0;
      	bodies[index].floating=0;
      	break;
      }

      // setup kinematic tree
      {
      	poseToTransform(j->parent_to_joint_origin_transform,bodies[index].Ttree);

        Vector3d zvec; zvec << 0,0,1;
        Vector3d joint_axis; joint_axis << j->axis.x, j->axis.y, j->axis.z;
        Vector3d axis = joint_axis.cross(zvec);
        double angle = acos(joint_axis.dot(zvec));
        double qx,qy,qz,qw;
        if (axis.squaredNorm()<.0001) //  then it's a scaling of the z axis.
          axis << 0,1,0;
        axis.normalize();
        qw=cos(angle/2.0);
        qx=axis(0)*sin(angle/2.0);
        qy=axis(1)*sin(angle/2.0);
        qz=axis(2)*sin(angle/2.0);

        bodies[index].T_body_to_joint <<
                qw*qw + qx*qx - qy*qy - qz*qz, 2*qx*qy - 2*qw*qz, 2*qx*qz + 2*qw*qy, 0,
                2*qx*qy + 2*qw*qz,  qw*qw + qy*qy - qx*qx - qz*qz, 2*qy*qz - 2*qw*qx, 0,
                2*qx*qz - 2*qw*qy, 2*qy*qz + 2*qw*qx, qw*qw + qz*qz - qx*qx - qy*qy, 0,
                0, 0, 0, 1;
      }

    } else { // no joint, this link is attached directly to the floating base
    	string jointname="base";
      map<string, int>::const_iterator jn=findWithSuffix(jointname_to_jointnum,jointname);
      //DEBUG
      //if (jn == jointname_to_jointnum.end()) {
        //for (auto p : jointname_to_jointnum) {
          //cout << p.first << endl;
        //}
      //}
      //END_DEBUG
    	if (jn == jointname_to_jointnum.end()) ROS_ERROR("can't find joint %s.  this shouldn't happen",jointname.c_str());
    	index = jn->second;
      map<string, int>::const_iterator dn=findWithSuffix(dofname_to_dofnum,"base_x");
      if (dn == dofname_to_dofnum.end()) ROS_ERROR("can't find dof base_x.  this shouldn't happen");
      _dofnum = dn->second;

    	// set up RigidBody (kinematics)
    	bodies[index].linkname = l->first;
    	bodies[index].jointname = jointname;

//    	cout << "body[" << index << "] linkname: " << bodies[index].linkname << ", jointname: " << bodies[index].jointname << endl;

    	bodies[index].parent = 0;
    	bodies[index].dofnum = _dofnum;
      bodies[index].floating = 1;
      // pitch is irrelevant
      bodies[index].Ttree = Matrix4d::Identity();
      bodies[index].T_body_to_joint = Matrix4d::Identity();

      // todo: set up featherstone structure (dynamics)
      parent[index-1] = -1;
      dofnum[index-1] = _dofnum;
    }

    if (l->second->collision) { // then at least one collision element exists
      // todo: iterate over all collision groups (not just "default")
      map<string, boost::shared_ptr<vector<boost::shared_ptr<urdf::Collision> > > >::iterator c_grp_it = l->second->collision_groups.find("default");
      for (size_t ic = 0;ic < c_grp_it->second->size();ic++)
      {
        vector<boost::shared_ptr<urdf::Collision> > *collisions = c_grp_it->second.get();
        for (vector<boost::shared_ptr<urdf::Collision> >::iterator citer = collisions->begin(); citer!=collisions->end(); citer++)
        {
          urdf::Collision * cptr = citer->get();
          if (!cptr) continue;

          Matrix4d T;
          poseToTransform(cptr->origin,T);
          DrakeCollision::Shape shape = DrakeCollision::Shape::UNKNOWN;

          int type = cptr->geometry->type;
          vector<double> params;
        	switch (type) {
        	case urdf::Geometry::BOX:
            {
          	  boost::shared_ptr<urdf::Box> box(boost::dynamic_pointer_cast<urdf::Box>(cptr->geometry));
              params.push_back(box->dim.x);
              params.push_back(box->dim.y);
              params.push_back(box->dim.z);
              shape = DrakeCollision::Shape::BOX;
            }
        		break;
        	case urdf::Geometry::SPHERE:
           	{
          		boost::shared_ptr<urdf::Sphere> sphere(boost::dynamic_pointer_cast<urdf::Sphere>(cptr->geometry));
              params.push_back(sphere->radius);
              shape = DrakeCollision::Shape::SPHERE;
          	}
        		break;
        	case urdf::Geometry::CYLINDER:
          	{
          		boost::shared_ptr<urdf::Cylinder> cyl(boost::dynamic_pointer_cast<urdf::Cylinder>(cptr->geometry));
              params.push_back(cyl->radius);
              params.push_back(cyl->length);
              shape = DrakeCollision::CYLINDER;
          	}
        		break;
        	case urdf::Geometry::MESH:
          	{
                  shape = DrakeCollision::Shape::MESH;
                  cerr << "Warning: mesh collision elements are not supported in URDFRigidBodyManipulator yet." << endl;
//              boost::shared_ptr<urdf::Mesh> mesh(boost::dynamic_pointer_cast<urdf::Mesh>(cptr->geometry));
          	}
        		break;
        	default:
        		cerr << "Link " << l->first << " has a collision element with an unknown type " << type << endl;
        		break;
        	}
                if (shape != DrakeCollision::Shape::MESH){
                  addCollisionElement(index,T,shape,params);
                }
        }
      }
      if (bodies[index].parent<0) {
        updateCollisionElements(index);  // update static objects only once - right here on load
      }
    }
  }

  compile();  
  return true;
}


URDFRigidBodyManipulator::~URDFRigidBodyManipulator(void)
{}

void URDFRigidBodyManipulator::draw(void)
{
	cerr << "Draw not implemented.  Did you mean to compile with BOT_VIS_SUPPORT?" << endl;
}

namespace urdf {

// pulled directly from ROS (actually from drc/software/robot_model/urdf_parser/)
boost::shared_ptr<ModelInterface> parseURDF(const string &xml_string)
{
  boost::shared_ptr<ModelInterface> model(new ModelInterface);
  model->clear();

  TiXmlDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());

  TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
  if (!robot_xml)
  {
    ROS_ERROR("Could not find the 'robot' element in the xml file");
    model.reset();
    return model;
  }

  // Get robot name
  const char *name = robot_xml->Attribute("name");
  if (!name)
  {
    ROS_ERROR("No name given for the robot.");
    model.reset();
    return model;
  }
  model->name_ = string(name);

  // Get all Material elements
  for (TiXmlElement* material_xml = robot_xml->FirstChildElement("material"); material_xml; material_xml = material_xml->NextSiblingElement("material"))
  {
    boost::shared_ptr<Material> material;
    material.reset(new Material);

    try {
      material->initXml(material_xml);
      if (model->getMaterial(material->name))
      {
        ROS_ERROR("material '%s' is not unique.", material->name.c_str());
        material.reset();
        model.reset();
        return model;
      }
      else
      {
        model->materials_.insert(make_pair(material->name,material));
        //ROS_DEBUG("successfully added a new material '%s'", material->name.c_str());
      }
    }
    catch (ParseError &e) {
      ROS_ERROR("material xml is not initialized correctly");
      material.reset();
      model.reset();
      return model;
    }
  }

  // Get all Link elements
  for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
  {
    boost::shared_ptr<Link> link;
    link.reset(new Link);

    try {
      link->initXml(link_xml);
      if (model->getLink(link->name))
      {
        ROS_ERROR("link '%s' is not unique.", link->name.c_str());
        model.reset();
        return model;
      }
      else
      {
        // set link visual material
        //ROS_DEBUG("setting link '%s' material", link->name.c_str());
        if (link->visual)
        {
          if (!link->visual->material_name.empty())
          {
            if (model->getMaterial(link->visual->material_name))
            {
              //ROS_DEBUG("setting link '%s' material to '%s'", link->name.c_str(),link->visual->material_name.c_str());
              link->visual->material = model->getMaterial( link->visual->material_name.c_str() );
            }
            else
            {
              if (link->visual->material)
              {
                //ROS_DEBUG("link '%s' material '%s' defined in Visual.", link->name.c_str(),link->visual->material_name.c_str());
                model->materials_.insert(make_pair(link->visual->material->name,link->visual->material));
              }
              else
              {
                ROS_ERROR("link '%s' material '%s' undefined.", link->name.c_str(),link->visual->material_name.c_str());
                model.reset();
                return model;
              }
            }
          }
        }

        model->links_.insert(make_pair(link->name,link));
        //ROS_DEBUG("successfully added a new link '%s'", link->name.c_str());
      }
    }
    catch (ParseError &e) {
      ROS_ERROR("link xml is not initialized correctly");
      model.reset();
      return model;
    }
  }
  if (model->links_.empty()){
    ROS_ERROR("No link elements found in urdf file");
    model.reset();
    return model;
  }

  // Get all Joint elements
  for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
  {
    boost::shared_ptr<Joint> joint;
    joint.reset(new Joint);

    if (joint->initXml(joint_xml))
    {
      if (model->getJoint(joint->name))
      {
        ROS_ERROR("joint '%s' is not unique.", joint->name.c_str());
        model.reset();
        return model;
      }
      else
      {
        model->joints_.insert(make_pair(joint->name,joint));
        //ROS_DEBUG("successfully added a new joint '%s'", joint->name.c_str());
      }
    }
    else
    {
      ROS_ERROR("joint xml is not initialized correctly");
      model.reset();
      return model;
    }
  }

  // every link has children links and joints, but no parents, so we create a
  // local convenience data structure for keeping child->parent relations
  map<string, string> parent_link_tree;
  parent_link_tree.clear();

  // building tree: name mapping
  if (!model->initTree(parent_link_tree))
  {
    ROS_ERROR("failed to build tree");
    model.reset();
    return model;
  }

  // find the root link
  if (!model->initRoot(parent_link_tree))
  {
    ROS_ERROR("failed to find root link");
    model.reset();
    return model;
  }
 
  return model;
}

}

/*
 * Checks if the string formed by concatenating `str` and `to_string(suffix)` 
 * is present in the range [first,last)
 */
bool isComboUnique(const string& str, int suffix,
							set<string>::iterator first,
							set<string>::iterator last)
{
	return find(first,last,str + to_string(suffix)) == last;
}

/* 
 * Finds an integer, `suffix`, such that the concatenation of `str` and 
 * `to_string(suffix)` is not present in the range [first,last)
 */
int findSuffix(const string& str, int suffix,
								set<string>::iterator first,
								set<string>::iterator last)
{
	while (!isComboUnique(str,suffix,first,last)) {
		++suffix;
	}
	return suffix;
}

/*
 * Modifies the string `name` by appending a numerical suffix such that the
 * modified string is not present in the set `name_set`
 */
void makeNameUnique(string& name, const set<string>& name_set)
{
  if (name_set.find(name)==name_set.end()) {
    return;
  }

  int suffix = findSuffix(name,2,name_set.begin(),name_set.end());
  name += to_string(suffix);
  return;
}

void setJointNum(boost::shared_ptr<urdf::ModelInterface> urdf_model, boost::shared_ptr<urdf::Joint> j, map<string, int> & jointname_to_jointnum, map<string, int> & dofname_to_dofnum, set<string>& joint_name_set,int& jointnum, int& dofnum)
{
  if (jointname_to_jointnum.find(j->name) != jointname_to_jointnum.end()) { // then i've already got a joint num
    return;  // this could happen if the parent comes after the child, and is allowed
  }

//  printf("setting joint num for %s\n", j->name.c_str());
  
  // recursively set parent num, then set j
  if (!j->parent_link_name.empty()) {
    map<string, boost::shared_ptr<urdf::Link> >::iterator plink = urdf_model->links_.find(j->parent_link_name);
    if (plink != urdf_model->links_.end()) {
      if (plink->second->parent_joint.get()) {
        // j has a parent
//        printf("  ");
        setJointNum(urdf_model,plink->second->parent_joint,jointname_to_jointnum,dofname_to_dofnum,joint_name_set,jointnum,dofnum);
      }
    }
  }

  makeNameUnique(j->name,joint_name_set);
  joint_name_set.insert(j->name);

  switch (j->type) {
    case urdf::Joint::REVOLUTE:
    case urdf::Joint::CONTINUOUS:
    case urdf::Joint::PRISMATIC:
    case urdf::Joint::FIXED:
      jointname_to_jointnum.insert(make_pair(j->name,jointnum++));
      dofname_to_dofnum.insert(make_pair(j->name,dofnum++));
      break;
    case urdf::Joint::FLOATING:
      ROS_ERROR("(internal) FLOATING joints implemented, but not tested yet.");
      // actually, i'll have to update the way that I set num_dof in the call to the RigidBodyManipulator constructor, too.
      jointname_to_jointnum.insert(make_pair(j->name,jointnum++));
      dofname_to_dofnum.insert(make_pair(j->name + "_x",dofnum++));
      dofname_to_dofnum.insert(make_pair(j->name + "_y",dofnum++));
      dofname_to_dofnum.insert(make_pair(j->name + "_z",dofnum++));
      dofname_to_dofnum.insert(make_pair(j->name + "_roll",dofnum++));
      dofname_to_dofnum.insert(make_pair(j->name + "_pitch",dofnum++));
      dofname_to_dofnum.insert(make_pair(j->name + "_yaw",dofnum++));
      break;
    case urdf::Joint::PLANAR:
      ROS_ERROR("PLANAR joints not supported yet.");
      break;
    default:
      ROS_ERROR("unsupported joint type %d for joint %s.", j->type, j->name.c_str());
      break;
      break;
  }
}

void setJointLimits(boost::shared_ptr<urdf::ModelInterface> urdf_model, boost::shared_ptr<urdf::Joint> j, const map<string, int> &dofname_to_dofnum, double* joint_limit_min, double* joint_limit_max)
{
  switch (j->type) {
    case urdf::Joint::REVOLUTE:
    case urdf::Joint::CONTINUOUS:
    case urdf::Joint::PRISMATIC:
    case urdf::Joint::FIXED:
      joint_limit_min[dofname_to_dofnum.at(j->name)] = j->limits->lower;
      joint_limit_max[dofname_to_dofnum.at(j->name)] = j->limits->upper;
      break;
    case urdf::Joint::FLOATING:
      joint_limit_min[dofname_to_dofnum.at(j->name)] = -1.0/0.0;
      joint_limit_max[dofname_to_dofnum.at(j->name)] = 1.0/0.0;
      break;
    default:
      ROS_ERROR("unsupported joint type %d for joint %s.", j->type, j->name.c_str());
      break;
  }
}

bool URDFRigidBodyManipulator::addURDFfromXML(const string &xml_string, const string &root_dir)
{
  // call ROS urdf parsing
	boost::shared_ptr<urdf::ModelInterface> _urdf_model;
	try {
		 _urdf_model = urdf::parseURDF(xml_string);
  } catch (urdf::ParseError &e) {
  	cerr << e.what() << endl;
  	return false;
  } catch (urdf::ParseError *e) {
  	cerr << e->what() << endl;
  	return false;
  }

  // produce a joint number for each joint where the parent has a
  // lower number than all of it's children.
  // NOTE:  The joint number does not necessarily match the number in
  // matlab.  I'm not removing fixed joints here.  I'm always adding the
  // the floating base joints.  But if these transformations are unused,
  // then they will simply stay at zero.
  map<string, int> jointname_to_jointnum;
  map<string, int> dofname_to_dofnum;

  // set up floating base
    // note: i see no harm in adding the floating base here (even if the drake version does not have one)
    // because the base will be set to 0 and it adds minimal expense to the kinematic calculations
  {  
    int dofnum=num_dof;
    int jointnum = num_bodies;
    string joint_name("base");
    makeNameUnique(joint_name,joint_name_set);
    joint_name_set.insert(joint_name);
    jointname_to_jointnum.insert(make_pair(joint_name,jointnum++));

    dofname_to_dofnum.insert(make_pair(joint_name+"_x",dofnum++));
    dofname_to_dofnum.insert(make_pair(joint_name+"_y",dofnum++));
    dofname_to_dofnum.insert(make_pair(joint_name+"_z",dofnum++));
    dofname_to_dofnum.insert(make_pair(joint_name+"_roll",dofnum++));
    dofname_to_dofnum.insert(make_pair(joint_name+"_pitch",dofnum++));
    dofname_to_dofnum.insert(make_pair(joint_name+"_yaw",dofnum++));
    
    for (map<string, boost::shared_ptr<urdf::Joint> >::iterator j=_urdf_model->joints_.begin(); j!=_urdf_model->joints_.end(); j++)
      setJointNum(_urdf_model,j->second,jointname_to_jointnum,dofname_to_dofnum,joint_name_set,jointnum,dofnum);
    

    //for (map<string, int>::iterator j=jointname_to_jointnum.begin(); j!=jointname_to_jointnum.end(); j++)
    //printf("%s : %d\n",j->first.c_str(),j->second);
    //DEBUG
    //for (auto p : dofname_to_dofnum) {
    //cout << "URDFRigidBodyManipulator::addURDFfromXML: p.first << " : " << p.second << endl; 
    //}
    //END_DEBUG
  }
  
  // now populate my model class
  addURDF(_urdf_model,jointname_to_jointnum,dofname_to_dofnum,root_dir);
  return true;
}

URDFRigidBodyManipulator* loadURDFfromXML(const string &xml_string, const string &root_dir)
{
#ifdef BOT_VIS_SUPPORT
  URDFRigidBodyManipulatorWBotVis* model = new URDFRigidBodyManipulatorWBotVis();
#else
  URDFRigidBodyManipulator* model = new URDFRigidBodyManipulator();
#endif
  model->addURDFfromXML(xml_string,root_dir);
  return model;
}

URDFRigidBodyManipulator* loadURDFfromFile(const string &urdf_filename)
{
	// urdf_filename can be a list of urdf files seperated by a :
#ifdef BOT_VIS_SUPPORT
  URDFRigidBodyManipulatorWBotVis* model = new URDFRigidBodyManipulatorWBotVis();
#else
  URDFRigidBodyManipulator* model = new URDFRigidBodyManipulator();
#endif

  string token;
  istringstream iss(urdf_filename);
  
  while (getline(iss,token,':')) {
    fstream xml_file(token.c_str(), fstream::in);
  	string xml_string;
    if (xml_file.is_open()) {
    	while ( xml_file.good() ) {
    		string line;
    		getline( xml_file, line);
    		xml_string += (line + "\n");
    	}
    	xml_file.close();
    } else {
    	cerr << "Could not open file ["<<urdf_filename.c_str()<<"] for parsing."<< endl;
    	return NULL;
    }

    string pathname="";
//    boost::filesystem::path mypath(urdf_filename);
//    if (!mypath.empty() && mypath.has_parent_path())		// note: if you see a segfault on has_parent_path(), then you probably tried to load the model without a parent path. (it shouldn't segfault, but a workaround is to load the model with a parent path, e.g. ./FallingBrick.urdf instead of FallingBrick.urdf)
//      pathname = mypath.parent_path().string();
    // I got too many segfaults with boost.  Doing it the old school way...
    size_t found = urdf_filename.find_last_of("/\\");
    if (found != string::npos) {
      pathname = urdf_filename.substr(0,found);
    }
    
    // parse URDF to get model
    model->addURDFfromXML(xml_string,pathname);
  }
  
  return model;
}
