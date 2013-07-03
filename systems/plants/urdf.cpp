
#include <stdio.h>
#include <stdarg.h>
#include <fstream>
#include <map>

#include "urdf.h"


#ifdef BOT_VIS_SUPPORT
#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
#endif

#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

using namespace std;

// Defines a method to load a model directly from a URDF file

void ROS_ERROR(const char* format, ...) {
  va_list vl;
  va_start(vl, format);
  vprintf(format, vl);
  va_end(vl);  
  printf("\n");
  exit(1);
}

void mexErrMsgIdandTxt(const char *errorid, const char *errormsg, ...)
{
  va_list vl;
  va_start(vl, errormsg);
  vprintf(errormsg, vl);
  va_end(vl);  
  printf("\n");
  exit(1);
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

URDFRigidBodyManipulator::URDFRigidBodyManipulator(boost::shared_ptr<urdf::ModelInterface> _urdf_model, map<string, int> jointname_to_jointnum, map<string,int> dofname_to_dofnum, const string & root_dir)
: 
  RigidBodyManipulator((int)dofname_to_dofnum.size(),-1,(int)jointname_to_jointnum.size()+1),
  joint_map(jointname_to_jointnum), dof_map(dofname_to_dofnum),
          urdf_model(_urdf_model)
{
	// set up floating base
    // note: i see no harm in adding the floating base here (even if the drake version does not have one)
    // because the base will be set to 0 and it adds minimal expense to the kinematic calculations
  {
    bodies[0].linkname = "world";
    bodies[0].parent = -1;
  }
  
  for (map<string, boost::shared_ptr<urdf::Link> >::iterator l=urdf_model->links_.begin(); l!=urdf_model->links_.end(); l++) {
    int index, _dofnum;
    if (l->second->parent_joint) {
    	boost::shared_ptr<urdf::Joint> j = l->second->parent_joint;
    	map<string, int>::iterator jn=joint_map.find(j->name);
    	if (jn == joint_map.end()) ROS_ERROR("can't find joint %s.  this shouldn't happen", j->name.c_str());
    	index = jn->second;
      map<string, int>::iterator dn=dof_map.find(j->name);
      if (dn == dof_map.end()) ROS_ERROR("can't find joint %s.  this shouldn't happen", j->name.c_str());
      _dofnum = dn->second;

    	bodies[index+1].linkname = l->first;
    	bodies[index+1].jointname = j->name;

    	{ // set up parent
    		map<string, boost::shared_ptr<urdf::Link> >::iterator pl=urdf_model->links_.find(j->parent_link_name);
    		if (pl == urdf_model->links_.end()) ROS_ERROR("can't find link %s.  this shouldn't happen", j->parent_link_name.c_str());

    		if (pl->second->parent_joint) {
    			boost::shared_ptr<urdf::Joint> pj = pl->second->parent_joint;
    			map<string, int>::iterator pjn=joint_map.find(pj->name);
    			if (pjn == joint_map.end()) ROS_ERROR("can't find joint %s.  this shouldn't happen", pj->name.c_str());

    			bodies[index+1].parent = pjn->second+1;
    			parent[index] = pjn->second;
    		} else { // the parent body is the floating base
    			bodies[index+1].parent = 1;
    		}
    	}

    	bodies[index+1].dofnum = _dofnum;
    	dofnum[index] = _dofnum;

    	// set pitch and floating
    	switch (j->type) {
    	case urdf::Joint::PRISMATIC:
    		pitch[index] = INF;
    		bodies[index+1].pitch = INF;
    		bodies[index+1].floating=0;
    		break;
      default:  // continuous, rotary, fixed, ...
      	pitch[index] = 0.0;
      	bodies[index+1].pitch = 0.0;
      	bodies[index+1].floating=0;
      	break;
      }

      // setup kinematic tree
      {
      	poseToTransform(j->parent_to_joint_origin_transform,bodies[index+1].Ttree);

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

        bodies[index+1].T_body_to_joint <<
                qw*qw + qx*qx - qy*qy - qz*qz, 2*qx*qy - 2*qw*qz, 2*qx*qz + 2*qw*qy, 0,
                2*qx*qy + 2*qw*qz,  qw*qw + qy*qy - qx*qx - qz*qz, 2*qy*qz - 2*qw*qx, 0,
                2*qx*qz - 2*qw*qy, 2*qy*qz + 2*qw*qx, qw*qw + qz*qz - qx*qx - qy*qy, 0,
                0, 0, 0, 1;
      }

    } else { // no joint, this link is attached directly to the floating base
    	index = 0;   // todo: update this when i support multiple (floating) bodies

    	// set up RigidBody (kinematics)
    	bodies[index+1].linkname = l->first;
    	bodies[index+1].jointname = "floating_base";
    	bodies[index+1].parent = 0;
    	bodies[index+1].dofnum = 0;  // todo: update this when i support multiple (floating) bodies
      bodies[index+1].floating = 1;
      // pitch is irrelevant
      bodies[index+1].Ttree = Matrix4d::Identity();
      bodies[index+1].T_body_to_joint = Matrix4d::Identity();

      // todo: set up featherstone structure (dynamics)
      parent[index] = -1;
      dofnum[index] = 0;
    }

#ifdef BOT_VIS_SUPPORT
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

          bool has_package = boost::find_first(mesh->filename,"package://");
          std::string fname = root_dir + "/" + mesh->filename;
          if (has_package) boost::replace_first(fname,"package:/","..");
          boost::filesystem::path mypath(fname);
          
          if (!boost::filesystem::exists(fname)) {
            cerr << "cannot find mesh file: " << fname;
            if (has_package)
              cerr << " (note: original mesh string had a package:// in it, and I haven't really implemented rospack yet)";
            cerr << endl;
            continue;
          }
          
          std::string ext = mypath.extension().native();
          boost::to_lower(ext);
          
          if (ext.compare(".dae")==0) {
            BotWavefrontModel* wavefront_model = bot_wavefront_model_create(fname.c_str());
            if (!wavefront_model) {
              cerr << "Error loading mesh: " << fname << endl;
            } else {
            	mesh_map.insert(make_pair(mesh->filename, wavefront_model));
            }
          } else {
          	// try changing the extension to dae and loading
          	if ( boost::filesystem::exists( mypath.replace_extension(".dae") ) ) {
          		BotWavefrontModel* wavefront_model = bot_wavefront_model_create(mypath.replace_extension(".dae").native().c_str());
              if (!wavefront_model) {
                cerr << "Error loading mesh: " << fname << endl;
              } else {
              	mesh_map.insert(make_pair(mesh->filename, wavefront_model));
              }
          	} else {
              cout << "Warning: Mesh " << mypath.native() << " ignored because it does not have extension .dae (nor can I find a juxtaposed file with a .dae extension)" << endl;
            }
          }
        }
      }
    }
#endif

#ifdef BULLET_COLLISION
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

        	RigidBody::CollisionObject co;
        	co.bt_obj = new btCollisionObject();
          int type = cptr->geometry->type;
        	switch (type) {
        	case urdf::Geometry::BOX:
            {
          	  boost::shared_ptr<urdf::Box> box(boost::dynamic_pointer_cast<urdf::Box>(cptr->geometry));
          	  co.bt_shape = new btBoxShape( btVector3(box->dim.x/2,box->dim.y/2,box->dim.z/2) );
            }
        		break;
        	case urdf::Geometry::SPHERE:
           	{
          		boost::shared_ptr<urdf::Sphere> sphere(boost::dynamic_pointer_cast<urdf::Sphere>(cptr->geometry));
          		co.bt_shape = new btSphereShape(sphere->radius) ;
          	}
        		break;
        	case urdf::Geometry::CYLINDER:
          	{
          		boost::shared_ptr<urdf::Cylinder> cyl(boost::dynamic_pointer_cast<urdf::Cylinder>(cptr->geometry));
          		co.bt_shape = new btCylinderShapeZ( btVector3(cyl->radius,cyl->radius,cyl->length/2) );
          	}
        		break;
        	case urdf::Geometry::MESH:
          	{
//              boost::shared_ptr<urdf::Mesh> mesh(boost::dynamic_pointer_cast<urdf::Mesh>(cptr->geometry));
              cerr << "Warning: mesh collision elements are not supported yet." << endl;
          	}
        		break;
        	default:
        		cerr << "Link " << l->first << " has a collision element with an unknown type " << type << endl;
        		break;
        	}

        	co.bt_obj->setCollisionShape(co.bt_shape);
        	poseToTransform(cptr->origin,co.T);

        	// add to the manipulator's collision world
        	bt_collision_world.addCollisionObject(co.bt_obj);

        	if (bodies[index+1].parent>=0) {
        		co.bt_obj->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
        		co.bt_obj->activate();
        	} else {
        		co.bt_obj->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
        	}

        	// add to the body
        	bodies[index+1].collision_objects.push_back(co);
        }
      }
      if (bodies[index+1].parent<0)
      	updateCollisionObjects(index+1);  // update static objects only once - right here on load
    }
#endif

  }


  compile();  
}

URDFRigidBodyManipulator::~URDFRigidBodyManipulator(void)
{
#ifdef BOT_VIS_SUPPORT
	for (map<string, BotWavefrontModel*>::iterator iter = mesh_map.begin(); iter!=mesh_map.end(); iter++)
		bot_wavefront_model_destroy(iter->second);
#endif
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

void setJointNum(boost::shared_ptr<urdf::ModelInterface> urdf_model, boost::shared_ptr<urdf::Joint> j, std::map<std::string, int> & jointname_to_jointnum, std::map<std::string, int> & dofname_to_dofnum)
{
  std::map<std::string, int>::iterator ans = jointname_to_jointnum.find(j->name);
  if (ans != jointname_to_jointnum.end()) // then i've already got a joint num
    return;

//  printf("setting joint num for %s\n", j->name.c_str());
  
  // recursively set parent num, then set j
  if (!j->parent_link_name.empty()) {
    std::map<std::string, boost::shared_ptr<urdf::Link> >::iterator plink = urdf_model->links_.find(j->parent_link_name);
    if (plink != urdf_model->links_.end()) {
      if (plink->second->parent_joint.get()) {
        // j has a parent
//        printf("  ");
        setJointNum(urdf_model,plink->second->parent_joint,jointname_to_jointnum,dofname_to_dofnum);
      }
    }
  }
  
  switch (j->type) {
    case urdf::Joint::REVOLUTE:
    case urdf::Joint::CONTINUOUS:
    case urdf::Joint::PRISMATIC:
    case urdf::Joint::FIXED:
      jointname_to_jointnum.insert(std::make_pair(j->name,(int)jointname_to_jointnum.size()));
      dofname_to_dofnum.insert(std::make_pair(j->name,(int)dofname_to_dofnum.size()));
      break;
    case urdf::Joint::FLOATING:
      ROS_ERROR("(internal) FLOATING joints implemented, but not tested yet.");
      // actually, i'll have to update the way that I set num_dof in the call to the RigidBodyManipulator constructor, too.
      jointname_to_jointnum.insert(std::make_pair(j->name,(int)jointname_to_jointnum.size()));
      dofname_to_dofnum.insert(std::make_pair(j->name + "_x",(int)dofname_to_dofnum.size()));
      dofname_to_dofnum.insert(std::make_pair(j->name + "_y",(int)dofname_to_dofnum.size()));
      dofname_to_dofnum.insert(std::make_pair(j->name + "_z",(int)dofname_to_dofnum.size()));
      dofname_to_dofnum.insert(std::make_pair(j->name + "_roll",(int)dofname_to_dofnum.size()));
      dofname_to_dofnum.insert(std::make_pair(j->name + "_pitch",(int)dofname_to_dofnum.size()));
      dofname_to_dofnum.insert(std::make_pair(j->name + "_yaw",(int)dofname_to_dofnum.size()));
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

URDFRigidBodyManipulator* loadURDFfromXML(const std::string &xml_string, const std::string &root_dir)
{
  // call ROS urdf parsing
	boost::shared_ptr<urdf::ModelInterface> urdf_model;
	try {
		 urdf_model = urdf::parseURDF(xml_string);
  } catch (urdf::ParseError &e) {
  	std::cerr << e.what() << endl;
  	return NULL;
  } catch (urdf::ParseError *e) {
  	std::cerr << e->what() << endl;
  	return NULL;
  }

  // produce a joint number for each joint where the parent has a
  // lower number than all of it's children.
  // NOTE:  The joint number does not necessarily match the number in
  // matlab.  I'm not removing fixed joints here.  I'm always adding the
  // the floating base joints.  But if these transformations are unused,
  // then they will simply stay at zero.
  std::map<std::string, int> jointname_to_jointnum;
  std::map<std::string, int> dofname_to_dofnum;
  {  
    int dofnum=0;
    jointname_to_jointnum.insert(std::make_pair("base",0));
    dofname_to_dofnum.insert(std::make_pair("base_x",dofnum++));
    dofname_to_dofnum.insert(std::make_pair("base_y",dofnum++));
    dofname_to_dofnum.insert(std::make_pair("base_z",dofnum++));
    dofname_to_dofnum.insert(std::make_pair("base_roll",dofnum++));
    dofname_to_dofnum.insert(std::make_pair("base_pitch",dofnum++));
    dofname_to_dofnum.insert(std::make_pair("base_yaw",dofnum++));
    
    for (std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator j=urdf_model->joints_.begin(); j!=urdf_model->joints_.end(); j++)
      setJointNum(urdf_model,j->second,jointname_to_jointnum,dofname_to_dofnum);
    
//    for (std::map<std::string, int>::iterator j=jointname_to_jointnum.begin(); j!=jointname_to_jointnum.end(); j++)
//      printf("%s : %d\n",j->first.c_str(),j->second);
  }
  
  // now populate my model class
  URDFRigidBodyManipulator* model = new URDFRigidBodyManipulator(urdf_model,jointname_to_jointnum,dofname_to_dofnum,root_dir);
  return model;
}

URDFRigidBodyManipulator* loadURDFfromFile(const std::string &urdf_filename)
{
  std::string xml_string;
  std::fstream xml_file(urdf_filename.c_str(), std::fstream::in);
  if (xml_file.is_open())
  {
    while ( xml_file.good() )
    {
      std::string line;
      std::getline( xml_file, line);
      xml_string += (line + "\n");
    }
    xml_file.close();
  }
  else
  {
    std::cerr << "Could not open file ["<<urdf_filename.c_str()<<"] for parsing."<< std::endl;
    return NULL;
  }
  
  boost::filesystem::path mypath(urdf_filename);
  std::string pathname;
  if (!mypath.empty() && mypath.has_parent_path())
  	pathname = mypath.parent_path().native();

  // parse URDF to get model
  return loadURDFfromXML(xml_string,pathname);
}
