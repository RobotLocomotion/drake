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

#ifndef URDF_INTERFACE_LINK_H
#define URDF_INTERFACE_LINK_H

#include <string>
#include <map>
#include <tinyxml.h>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include "joint.h"
#include "color.h"

namespace urdf{

class Geometry
{
public:
  enum {SPHERE, BOX, CYLINDER, MESH} type;

  virtual void initXml(TiXmlElement *) = 0;

};

class Sphere : public Geometry
{
public:
  Sphere() { this->clear(); };
  double radius;

  void clear()
  {
    radius = 0;
  };
  void initXml(TiXmlElement *);
};

class Box : public Geometry
{
public:
  Box() { this->clear(); };
  Vector3 dim;

  void clear()
  {
    this->dim.clear();
  };
  void initXml(TiXmlElement *);
};

class Cylinder : public Geometry
{
public:
  Cylinder() { this->clear(); };
  double length;
  double radius;

  void clear()
  {
    length = 0;
    radius = 0;
  };
  void initXml(TiXmlElement *);
};

class Mesh : public Geometry
{
public:
  Mesh() { this->clear(); };
  std::string filename;
  Vector3 scale;

  void clear()
  {
    filename.clear();
    // default scale
    scale.x = 1;
    scale.y = 1;
    scale.z = 1;
  };
  void initXml(TiXmlElement *);
  bool fileExists(std::string filename);
};

class Material
{
public:
  Material() { this->clear(); };
  std::string name;
  std::string texture_filename;
  Color color;

  void clear()
  {
    color.clear();
    texture_filename.clear();
    name.clear();
  };
  void initXml(TiXmlElement* config);
};

class Inertial
{
public:
  Inertial() { this->clear(); };
  Pose origin;
  double mass;
  double ixx,ixy,ixz,iyy,iyz,izz;

  void clear()
  {
    origin.clear();
    mass = 0;
    ixx = ixy = ixz = iyy = iyz = izz = 0;
  };
  void initXml(TiXmlElement* config);
};

class Visual
{
public:
  Visual() { this->clear(); };
  Pose origin;
  boost::shared_ptr<Geometry> geometry;

  std::string material_name;
  boost::shared_ptr<Material> material;

  void clear()
  {
    origin.clear();
    material_name.clear();
    material.reset();
    geometry.reset();
    this->group_name.clear();
  };
  void initXml(TiXmlElement* config);
  std::string group_name;
};

class Collision
{
public:
  Collision() { this->clear(); };
  Pose origin;
  boost::shared_ptr<Geometry> geometry;

  void clear()
  {
    origin.clear();
    geometry.reset();
    this->group_name.clear();
  };
  void initXml(TiXmlElement* config);
  std::string group_name;
};


class Link
{
public:
  Link() { this->clear(); };

  std::string name;

  /// inertial element
  boost::shared_ptr<Inertial> inertial;

  /// visual element
  boost::shared_ptr<Visual> visual;

  /// collision element
  boost::shared_ptr<Collision> collision;

  /// a collection of visual elements, keyed by a string tag called "group"
  std::map<std::string, boost::shared_ptr<std::vector<boost::shared_ptr<Visual> > > > visual_groups;

  /// a collection of collision elements, keyed by a string tag called "group"
  std::map<std::string, boost::shared_ptr<std::vector<boost::shared_ptr<Collision> > > > collision_groups;

  /// Parent Joint element
  ///   explicitly stating "parent" because we want directional-ness for tree structure
  ///   every link can have one parent
  boost::shared_ptr<Joint> parent_joint;

  std::vector<boost::shared_ptr<Joint> > child_joints;
  std::vector<boost::shared_ptr<Link> > child_links;

  void initXml(TiXmlElement* config);

  boost::shared_ptr<Link> getParent() const
  {return parent_link_.lock();};

  void setParent(boost::shared_ptr<Link> parent);

  void clear()
  {
    this->name.clear();
    this->inertial.reset();
    this->visual.reset();
    this->collision.reset();
    this->parent_joint.reset();
    this->child_joints.clear();
    this->child_links.clear();
    this->collision_groups.clear();
  };
  void setParentJoint(boost::shared_ptr<Joint> child);
  void addChild(boost::shared_ptr<Link> child);
  void addChildJoint(boost::shared_ptr<Joint> child);

  void addVisual(std::string group_name, boost::shared_ptr<Visual> visual);
  boost::shared_ptr<std::vector<boost::shared_ptr<Visual > > > getVisuals(const std::string& group_name) const;
  void addCollision(std::string group_name, boost::shared_ptr<Collision> collision);
  boost::shared_ptr<std::vector<boost::shared_ptr<Collision > > > getCollisions(const std::string& group_name) const;
private:
  boost::weak_ptr<Link> parent_link_;

};




}

#endif
