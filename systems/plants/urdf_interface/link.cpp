/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */


#include <urdf_interface/link.h>
#include <fstream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <urdf_interface/exceptions.h>

namespace urdf{

boost::shared_ptr<Geometry> parseGeometry(TiXmlElement *g)
{
  boost::shared_ptr<Geometry> geom;
  if (!g) return geom;

  TiXmlElement *shape = g->FirstChildElement();
  if (!shape)
  {
    //ROS_ERROR("Geometry tag contains no child element.");
    return geom;
  }

  std::string type_name = shape->ValueStr();
  if (type_name == "sphere")
    geom.reset(new Sphere);
  else if (type_name == "box")
    geom.reset(new Box);
  else if (type_name == "cylinder")
    geom.reset(new Cylinder);
  else if (type_name == "mesh")
    geom.reset(new Mesh);
  else
  {
    //ROS_ERROR("Unknown geometry type '%s'", type_name.c_str());
    return geom;
  }

  // clear geom object when fails to initialize
  try {
    geom->initXml(shape);
  }
  catch (ParseError &e) {
    geom.reset();
    throw e.addMessage("failed to parse Geometry from shape");
  }

  return geom;
}

void Material::initXml(TiXmlElement *config)
{
  bool has_rgb = false;
  bool has_filename = false;

  this->clear();

  try {
    config->Attribute("name");
  }
  catch (ParseError &e) {
    throw e.addMessage("Material must contain a name attribute");
  }

  this->name = config->Attribute("name");

  // texture
  TiXmlElement *t = config->FirstChildElement("texture");
  if (t)
  {
    if (t->Attribute("filename"))
    {
      this->texture_filename = t->Attribute("filename");
      has_filename = true;
    }
  }

  // color
  TiXmlElement *c = config->FirstChildElement("color");
  if (c)
  {
    if (c->Attribute("rgba")) {

      try {
        this->color.init(c->Attribute("rgba"));
      }
      catch (ParseError &e) {
        this->color.clear();
        throw e.addMessage("Material ["+this->name+"] has malformed color rgba values.");
      }

      has_rgb = true;
    }
  }

  if (!has_rgb && !has_filename) {
    ParseError e;
    if (!has_rgb) e.addMessage("Material ["+this->name+"] color has no rgba");
    if (!has_filename) e.addMessage("Material ["+this->name+"] not defined in file");
    throw e;
  }
}

void Inertial::initXml(TiXmlElement *config)
{
  this->clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o)
  {
    try {
      this->origin.initXml(o);
    }
    catch (ParseError &e) {
      this->origin.clear();
      throw e.addMessage("Inertial has a malformed origin tag");
    }
  }

  TiXmlElement *mass_xml = config->FirstChildElement("mass");
  if (!mass_xml)
  {
    throw ParseError("Inertial element must have a mass element");
  }
  if (!mass_xml->Attribute("value"))
  {
    throw ParseError("Inertial: mass element must have value attributes");
  }

  try
  {
    mass = boost::lexical_cast<double>(mass_xml->Attribute("value"));
  }
  catch (boost::bad_lexical_cast &e)
  {
    std::stringstream stm;
    stm << "mass [" << mass_xml->Attribute("value")
        << "] is not a float";
    throw ParseError(stm.str());
  }

  TiXmlElement *inertia_xml = config->FirstChildElement("inertia");
  if (!inertia_xml)
  {
    throw ParseError("Inertial element must have inertia element");
  }
  if (!(inertia_xml->Attribute("ixx") && inertia_xml->Attribute("ixy") && inertia_xml->Attribute("ixz") &&
        inertia_xml->Attribute("iyy") && inertia_xml->Attribute("iyz") &&
        inertia_xml->Attribute("izz")))
  {
    throw ParseError("Inertial: inertia element must have ixx,ixy,ixz,iyy,iyz,izz attributes");
  }
  try
  {
    ixx  = boost::lexical_cast<double>(inertia_xml->Attribute("ixx"));
    ixy  = boost::lexical_cast<double>(inertia_xml->Attribute("ixy"));
    ixz  = boost::lexical_cast<double>(inertia_xml->Attribute("ixz"));
    iyy  = boost::lexical_cast<double>(inertia_xml->Attribute("iyy"));
    iyz  = boost::lexical_cast<double>(inertia_xml->Attribute("iyz"));
    izz  = boost::lexical_cast<double>(inertia_xml->Attribute("izz"));
  }
  catch (boost::bad_lexical_cast &e)
  {
    std::stringstream stm;
    stm << "one of the inertia elements is not a valid double:"
        << " ixx [" << inertia_xml->Attribute("ixx") << "]"
        << " ixy [" << inertia_xml->Attribute("ixy") << "]"
        << " ixz [" << inertia_xml->Attribute("ixz") << "]"
        << " iyy [" << inertia_xml->Attribute("iyy") << "]"
        << " iyz [" << inertia_xml->Attribute("iyz") << "]"
        << " izz [" << inertia_xml->Attribute("izz") << "]";
    throw ParseError(stm.str());
  }

}

void Visual::initXml(TiXmlElement *config)
{
  this->clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o) {
    try {
      this->origin.initXml(o);
    }
    catch (ParseError &e) {
      this->origin.clear();
      throw e.addMessage("Visual has a malformed origin tag");
    }
  }

  // Geometry
  TiXmlElement *geom = config->FirstChildElement("geometry");
  this->geometry = parseGeometry(geom);
  if (!this->geometry) {
    throw ParseError("Malformed geometry for Visual element");
  }

  // Material
  TiXmlElement *mat = config->FirstChildElement("material");
  if (mat) {
    // get material name
    if (!mat->Attribute("name")) {
      throw ParseError("Visual material must contain a name attribute");
    }
    this->material_name = mat->Attribute("name");

    // try to parse material element in place
    this->material.reset(new Material);
    try {
      this->material->initXml(mat);
    }
    catch (ParseError &e) {
      this->material.reset();
      //e.addMessage("INFO: Could not parse material element in Visual block, maybe defined outside.  resetting material, but not throwing.");
    }
  }

  // Group Tag (optional)
  // collision blocks without a group tag are designated to the "default" group
  const char *group_name_char = config->Attribute("group");
  if (!group_name_char)
    group_name = std::string("default");
  else
    group_name = std::string(group_name_char);
}

void Collision::initXml(TiXmlElement* config)
{  
  this->clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o) {
    try {
      this->origin.initXml(o);
    }
    catch (ParseError &e)
    {
      this->origin.clear();
      throw e.addMessage("Collision has a malformed origin tag");
    }
  }

  // Geometry
  TiXmlElement *geom = config->FirstChildElement("geometry");
  this->geometry = parseGeometry(geom);
  if (!this->geometry)
  {
    throw ParseError("Malformed geometry for Collision element");
  }

  // Group Tag (optional)
  // collision blocks without a group tag are designated to the "default" group
  const char *group_name_char = config->Attribute("group");
  if (!group_name_char)
    group_name = std::string("default");
  else
    group_name = std::string(group_name_char);
}

void Sphere::initXml(TiXmlElement *c)
{
  this->clear();

  this->type = SPHERE;
  if (!c->Attribute("radius"))
  {
    throw ParseError("Sphere shape must have a radius attribute");
  }

  try
  {
    radius = boost::lexical_cast<double>(c->Attribute("radius"));
  }
  catch (boost::bad_lexical_cast &e)
  {
    std::stringstream stm;
    stm << "radius [" << c->Attribute("radius") << "] is not a valid float";
    throw ParseError(stm.str());
  }

}

void Box::initXml(TiXmlElement *c)
{
  this->clear();

  this->type = BOX;
  if (!c->Attribute("size"))
  {
    throw ParseError("Box shape has no size attribute");
  }
  try
  {
    this->dim.init(c->Attribute("size"));
  }
  catch (ParseError &e)
  {
    this->dim.clear();
    std::stringstream stm;
    stm << "Box shape has malformed size attribute ["
        << c->Attribute("size")
        << "].";
    throw e.addMessage(stm.str());
  }
}

void Cylinder::initXml(TiXmlElement *c)
{
  this->clear();

  this->type = CYLINDER;
  if (!c->Attribute("length") ||
      !c->Attribute("radius"))
  {
    throw ParseError("Cylinder shape must have both length and radius attributes");
  }

  try
  {
    length = boost::lexical_cast<double>(c->Attribute("length"));
  }
  catch (boost::bad_lexical_cast &e)
  {
    std::stringstream stm;
    stm << "length [" << c->Attribute("length") << "] is not a valid float";
    throw ParseError(stm.str());
  }

  try
  {
    radius = boost::lexical_cast<double>(c->Attribute("radius"));
  }
  catch (boost::bad_lexical_cast &e)
  {
    std::stringstream stm;
    stm << "radius [" << c->Attribute("radius") << "] is not a valid float";
    throw ParseError(stm.str());
  }
}


void Mesh::initXml(TiXmlElement *c)
{
  this->clear();

  this->type = MESH;
  if (!c->Attribute("filename")) {
    throw ParseError("Mesh must contain a filename attribute");
  }

  filename = c->Attribute("filename");

  if (c->Attribute("scale")) {
    try {
      this->scale.init(c->Attribute("scale"));
    }
    catch (ParseError &e) {
      this->scale.clear();
      throw e.addMessage("Mesh scale was specified, but could not be parsed");
    }
  }
  else
  {
    //("Mesh scale was not specified, default to (1,1,1)");
  }
}


void Link::initXml(TiXmlElement* config)
{
  
  this->clear();

  const char *name_char = config->Attribute("name");
  if (!name_char)
  {
    throw ParseError("No name given for the link.");
  }
  name = std::string(name_char);

  // Inertial (optional)
  TiXmlElement *i = config->FirstChildElement("inertial");
  if (i)
  {
    inertial.reset(new Inertial);
    try {
      inertial->initXml(i);
    }
    catch (ParseError &e) {
      std::stringstream stm;
      stm << "Could not parse inertial element for Link [" << this->name << "]";
      throw e.addMessage(stm.str());
    }
  }

  // Multiple Visuals (optional)
  for (TiXmlElement* vis_xml = config->FirstChildElement("visual"); vis_xml; vis_xml = vis_xml->NextSiblingElement("visual"))
  {

    boost::shared_ptr<Visual> vis;
    vis.reset(new Visual);

    try {
      vis->initXml(vis_xml);
      boost::shared_ptr<std::vector<boost::shared_ptr<Visual > > > viss = this->getVisuals(vis->group_name);
      if (!viss)
      {
        // group does not exist, create one and add to map
        viss.reset(new std::vector<boost::shared_ptr<Visual > >);
        // new group name, create vector, add vector to map and add Visual to the vector
        this->visual_groups.insert(make_pair(vis->group_name,viss));
        //ROS_DEBUG("successfully added a new visual group name '%s'",vis->group_name.c_str());
      }

      // group exists, add Visual to the vector in the map
      viss->push_back(vis);
      //ROS_DEBUG("successfully added a new visual under group name '%s'",vis->group_name.c_str());
    }
    catch (ParseError &e) {
      vis.reset();
      std::stringstream stm;
      stm << "Could not parse visual element for Link [" << this->name << "]";
      throw e.addMessage(stm.str());
    }
  }

  // Visual (optional)
  // Assign one single default visual pointer from the visual_groups map
  this->visual.reset();
  boost::shared_ptr<std::vector<boost::shared_ptr<Visual > > > default_visual = this->getVisuals("default");
  if (!default_visual)
  {
    //("No 'default' visual group for Link '%s'", this->name.c_str());
  }
  else if (default_visual->empty())
  {
    //("'default' visual group is empty for Link '%s'", this->name.c_str());
  }
  else
  {
    if (default_visual->size() > 1)
    {
      //("'default' visual group has %d visuals for Link '%s', taking the first one as default",(int)default_visual->size(), this->name.c_str());
    }
    this->visual = (*default_visual->begin());
  }


  // Multiple Collisions (optional)
  for (TiXmlElement* col_xml = config->FirstChildElement("collision"); col_xml; col_xml = col_xml->NextSiblingElement("collision"))
  {
    boost::shared_ptr<Collision> col;
    col.reset(new Collision);
    try {
      col->initXml(col_xml);

      boost::shared_ptr<std::vector<boost::shared_ptr<Collision > > > cols = this->getCollisions(col->group_name);  
      
      if (!cols)
      {
        // group does not exist, create one and add to map
        cols.reset(new std::vector<boost::shared_ptr<Collision > >);
        // new group name, create vector, add vector to map and add Collision to the vector
        this->collision_groups.insert(make_pair(col->group_name,cols));
        //ROS_DEBUG("successfully added a new collision group name '%s'",col->group_name.c_str());
      }

      // group exists, add Collision to the vector in the map
      cols->push_back(col);
      //ROS_DEBUG("successfully added a new collision under group name '%s'",col->group_name.c_str());
    }
    catch (ParseError &e) {
      col.reset();
      std::stringstream stm;
      stm << "Could not parse collision element for Link [" << this->name << "]";
      throw ParseError(stm.str());
    }
  }
  

  // Collision (optional)
  // Assign one single default collision pointer from the collision_groups map
  this->collision.reset();
  boost::shared_ptr<std::vector<boost::shared_ptr<Collision > > > default_collision = this->getCollisions("default");

  if (!default_collision)
  {
    //ROS_DEBUG("No 'default' collision group for Link '%s'", this->name.c_str());
  }
  else if (default_collision->empty())
  {
    //ROS_DEBUG("'default' collision group is empty for Link '%s'", this->name.c_str());
  }
  else
  {
    if (default_collision->size() > 1)
    {
      //ROS_WARN("'default' collision group has %d collisions for Link '%s', taking the first one as default",(int)default_collision->size(), this->name.c_str());
    }
    this->collision = (*default_collision->begin());
  }
}

void Link::addVisual(std::string group_name, boost::shared_ptr<Visual> visual)
{
  boost::shared_ptr<std::vector<boost::shared_ptr<Visual > > > viss = this->getVisuals(group_name);
  if (!viss)
  {
    // group does not exist, create one and add to map
    viss.reset(new std::vector<boost::shared_ptr<Visual > >);
    // new group name, create vector, add vector to map and add Visual to the vector
    this->visual_groups.insert(make_pair(group_name,viss));
    //ROS_DEBUG("successfully added a new visual group name '%s'",group_name.c_str());
  }

  // group exists, add Visual to the vector in the map
  std::vector<boost::shared_ptr<Visual > >::iterator vis_it = find(viss->begin(),viss->end(),visual);
  if (vis_it != viss->end())
  {
    //ROS_WARN("attempted to add a visual that already exists under group name '%s', skipping.",group_name.c_str());
  }
  else
    viss->push_back(visual);
  //ROS_DEBUG("successfully added a new visual under group name '%s'",group_name.c_str());

}

boost::shared_ptr<std::vector<boost::shared_ptr<Visual > > > Link::getVisuals(const std::string& group_name) const
{
  boost::shared_ptr<std::vector<boost::shared_ptr<Visual > > > ptr;
  if (this->visual_groups.find(group_name) == this->visual_groups.end())
    ptr.reset();
  else
    ptr = this->visual_groups.find(group_name)->second;
  return ptr;
}


void Link::addCollision(std::string group_name, boost::shared_ptr<Collision> collision)
{
  boost::shared_ptr<std::vector<boost::shared_ptr<Collision > > > viss = this->getCollisions(group_name);
  if (!viss)
  {
    // group does not exist, create one and add to map
    viss.reset(new std::vector<boost::shared_ptr<Collision > >);
    // new group name, create vector, add vector to map and add Collision to the vector
    this->collision_groups.insert(make_pair(group_name,viss));
    //ROS_DEBUG("successfully added a new collision group name '%s'",group_name.c_str());
  }

  // group exists, add Collision to the vector in the map
  std::vector<boost::shared_ptr<Collision > >::iterator vis_it = find(viss->begin(),viss->end(),collision);
  if (vis_it != viss->end())
  {
    //ROS_WARN("attempted to add a collision that already exists under group name '%s', skipping.",group_name.c_str());
  }
  else
    viss->push_back(collision);
  //ROS_DEBUG("successfully added a new collision under group name '%s'",group_name.c_str());

}

boost::shared_ptr<std::vector<boost::shared_ptr<Collision > > > Link::getCollisions(const std::string& group_name) const
{
  boost::shared_ptr<std::vector<boost::shared_ptr<Collision > > > ptr;
  if (this->collision_groups.find(group_name) == this->collision_groups.end())
    ptr.reset();
  else
    ptr = this->collision_groups.find(group_name)->second;
  return ptr;
}

void Link::setParent(boost::shared_ptr<Link> parent)
{
  this->parent_link_ = parent;
  //ROS_DEBUG("set parent Link '%s' for Link '%s'", parent->name.c_str(), this->name.c_str());
}

void Link::setParentJoint(boost::shared_ptr<Joint> parent)
{
  this->parent_joint = parent;
  //ROS_DEBUG("set parent joint '%s' to Link '%s'",  parent->name.c_str(), this->name.c_str());
}

void Link::addChild(boost::shared_ptr<Link> child)
{
  this->child_links.push_back(child);
  //ROS_DEBUG("added child Link '%s' to Link '%s'",  child->name.c_str(), this->name.c_str());
}

void Link::addChildJoint(boost::shared_ptr<Joint> child)
{
  this->child_joints.push_back(child);
  //ROS_DEBUG("added child Joint '%s' to Link '%s'", child->name.c_str(), this->name.c_str());
}



}

