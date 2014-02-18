/*********************************************************************
* Software Ligcense Agreement (BSD License)
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

/* Author: John Hsu */

#include <stdio.h>
#include <stdarg.h>
#include <sstream>
#include <boost/lexical_cast.hpp>

#include "exceptions.h"
#include "joint.h"

void ROS_ERROR(const char* format, ...) {
  va_list vl;
  va_start(vl, format);
  vprintf(format, vl);
  va_end(vl);
  printf("\n");
  exit(1);
}

namespace urdf{

bool JointDynamics::initXml(TiXmlElement* config)
{
  this->clear();

  // Get joint damping
  const char* damping_str = config->Attribute("damping");
  if (damping_str == NULL){
    //ROS_DEBUG("joint dynamics: no damping, defaults to 0");
    this->damping = 0;
  }
  else
  {
    try
    {
      this->damping = boost::lexical_cast<double>(damping_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      ROS_ERROR("damping value (%s) is not a float",damping_str);
      return false;
    }
  }

  // Get joint friction
  const char* friction_str = config->Attribute("friction");
  if (friction_str == NULL){
    //ROS_DEBUG("joint dynamics: no friction, defaults to 0");
    this->friction = 0;
  }
  else
  {
    try
    {
      this->friction = boost::lexical_cast<double>(friction_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      ROS_ERROR("friction value (%s) is not a float",friction_str);
      return false;
    }
  }

  if (damping_str == NULL && friction_str == NULL)
  {
    ROS_ERROR("joint dynamics element specified with no damping and no friction");
    return false;
  }
  else{
    //ROS_DEBUG("joint dynamics: damping %f and friction %f", damping, friction);
    return true;
  }
}

bool JointLimits::initXml(TiXmlElement* config)
{
  this->clear();

  // Get lower joint limit
  const char* lower_str = config->Attribute("lower");
  if (lower_str == NULL){
    //ROS_DEBUG("joint limit: no lower, defaults to 0");
    this->lower = 0;
  }
  else
  {
    try
    {
      this->lower = boost::lexical_cast<double>(lower_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      ROS_ERROR("lower value (%s) is not a float",lower_str);
      return false;
    }
  }

  // Get upper joint limit
  const char* upper_str = config->Attribute("upper");
  if (upper_str == NULL){
    //ROS_DEBUG("joint limit: no upper, , defaults to 0");
    this->upper = 0;
  }
  else
  {
    try
    {
      this->upper = boost::lexical_cast<double>(upper_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      ROS_ERROR("upper value (%s) is not a float",upper_str);
      return false;
    }
  }

  // Get joint effort limit
  const char* effort_str = config->Attribute("effort");
  if (effort_str == NULL){
    ROS_ERROR("joint limit: no effort");
    return false;
  }
  else
  {
    try
    {
      this->effort = boost::lexical_cast<double>(effort_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      //ROS_ERROR("effort value (%s) is not a float",effort_str);
      return false;
    }
  }

  // Get joint velocity limit
  const char* velocity_str = config->Attribute("velocity");
  if (velocity_str == NULL){
    ROS_ERROR("joint limit: no velocity");
    return false;
  }
  else
  {
    try
    {
      this->velocity = boost::lexical_cast<double>(velocity_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      ROS_ERROR("velocity value (%s) is not a float",velocity_str);
      return false;
    }
  }

  return true;
}

bool JointSafety::initXml(TiXmlElement* config)
{
  this->clear();

  // Get soft_lower_limit joint limit
  const char* soft_lower_limit_str = config->Attribute("soft_lower_limit");
  if (soft_lower_limit_str == NULL)
  {
    //ROS_DEBUG("joint safety: no soft_lower_limit, using default value");
    this->soft_lower_limit = 0;
  }
  else
  {
    try
    {
      this->soft_lower_limit = boost::lexical_cast<double>(soft_lower_limit_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      ROS_ERROR("soft_lower_limit value (%s) is not a float",soft_lower_limit_str);
      return false;
    }
  }

  // Get soft_upper_limit joint limit
  const char* soft_upper_limit_str = config->Attribute("soft_upper_limit");
  if (soft_upper_limit_str == NULL)
  {
    //ROS_DEBUG("joint safety: no soft_upper_limit, using default value");
    this->soft_upper_limit = 0;
  }
  else
  {
    try
    {
      this->soft_upper_limit = boost::lexical_cast<double>(soft_upper_limit_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      ROS_ERROR("soft_upper_limit value (%s) is not a float",soft_upper_limit_str);
      return false;
    }
  }

  // Get k_position_ safety "position" gain - not exactly position gain
  const char* k_position_str = config->Attribute("k_position");
  if (k_position_str == NULL)
  {
    //ROS_DEBUG("joint safety: no k_position, using default value");
    this->k_position = 0;
  }
  else
  {
    try
    {
      this->k_position = boost::lexical_cast<double>(k_position_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      ROS_ERROR("k_position value (%s) is not a float",k_position_str);
      return false;
    }
  }
  // Get k_velocity_ safety velocity gain
  const char* k_velocity_str = config->Attribute("k_velocity");
  if (k_velocity_str == NULL)
  {
    ROS_ERROR("joint safety: no k_velocity");
    return false;
  }
  else
  {
    try
    {
      this->k_velocity = boost::lexical_cast<double>(k_velocity_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      ROS_ERROR("k_velocity value (%s) is not a float",k_velocity_str);
      return false;
    }
  }

  return true;
}

bool JointCalibration::initXml(TiXmlElement* config)
{
  this->clear();

  // Get rising edge position
  const char* rising_position_str = config->Attribute("rising");
  if (rising_position_str == NULL)
  {
    //ROS_DEBUG("joint calibration: no rising, using default value");
    this->rising.reset();
  }
  else
  {
    try
    {
      this->rising.reset(new double(boost::lexical_cast<double>(rising_position_str)));
    }
    catch (boost::bad_lexical_cast &e)
    {
      ROS_ERROR("risingvalue (%s) is not a float",rising_position_str);
      return false;
    }
  }

  // Get falling edge position
  const char* falling_position_str = config->Attribute("falling");
  if (falling_position_str == NULL)
  {
    //ROS_DEBUG("joint calibration: no falling, using default value");
    this->falling.reset();
  }
  else
  {
    try
    {
      this->falling.reset(new double(boost::lexical_cast<double>(falling_position_str)));
    }
    catch (boost::bad_lexical_cast &e)
    {
      ROS_ERROR("fallingvalue (%s) is not a float",falling_position_str);
      return false;
    }
  }

  return true;
}

bool JointMimic::initXml(TiXmlElement* config)
{
  this->clear();

  // Get name of joint to mimic
  const char* joint_name_str = config->Attribute("joint");

  if (joint_name_str == NULL)
  {
    //ROS_ERROR("joint mimic: no mimic joint specified");
    //return false;
  }
  else
     this->joint_name = joint_name_str;
  
  // Get mimic multiplier
  const char* multiplier_str = config->Attribute("multiplier");

  if (multiplier_str == NULL)
  {
    //ROS_DEBUG("joint mimic: no multiplier, using default value of 1");
    this->multiplier = 1;    
  }
  else
  {
    try
    {
      this->multiplier = boost::lexical_cast<double>(multiplier_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      ROS_ERROR("multiplier value (%s) is not a float",multiplier_str);
      return false;
    }
  }

  
  // Get mimic offset
  const char* offset_str = config->Attribute("offset");
  if (offset_str == NULL)
  {
    //ROS_DEBUG("joint mimic: no offset, using default value of 0");
    this->offset = 0;
  }
  else
  {
    try
    {
      this->offset = boost::lexical_cast<double>(offset_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      ROS_ERROR("offset value (%s) is not a float",offset_str);
      return false;
    }
  }

  return true;
}

bool Joint::initXml(TiXmlElement* config)
{
  this->clear();

  // Get Joint Name
  const char *name = config->Attribute("name");
  if (!name)
  {
    ROS_ERROR("unnamed joint found");
    return false;
  }
  this->name = name;

  // Get transform from Parent Link to Joint Frame
  TiXmlElement *origin_xml = config->FirstChildElement("origin");
  if (!origin_xml)
  {
    //ROS_DEBUG("Joint '%s' missing origin tag under parent describing transform from Parent Link to Joint Frame, (using Identity transform).", this->name.c_str());
    this->parent_to_joint_origin_transform.clear();
  }
  else
  {
    try {
      this->parent_to_joint_origin_transform.initXml(origin_xml);
    }
    catch (ParseError &e) {
      this->parent_to_joint_origin_transform.clear();
      std::stringstream stm;
      stm << "Malformed parent origin element for joint [" << this->name << "]";
      throw ParseError(stm.str());
    }
  }

  // Get Parent Link
  TiXmlElement *parent_xml = config->FirstChildElement("parent");
  if (parent_xml)
  {
    const char *pname = parent_xml->Attribute("link");
    if (!pname)
    {
      //ROS_INFO("no parent link name specified for Joint link '%s'. this might be the root?", this->name.c_str());
    }
    else
    {
      this->parent_link_name = std::string(pname);

    }
  }

  // Get Child Link
  TiXmlElement *child_xml = config->FirstChildElement("child");
  if (child_xml)
  {
    const char *pname = child_xml->Attribute("link");
    if (!pname)
    {
      //ROS_INFO("no child link name specified for Joint link '%s'.", this->name.c_str());
    }
    else
    {
      this->child_link_name = std::string(pname);

    }
  }

  // Get Joint type
  const char* type_char = config->Attribute("type");
  if (!type_char)
  {
    ROS_ERROR("joint '%s' has no type, check to see if it's a reference.", this->name.c_str());
    return false;
  }
  std::string type_str = type_char;
  if (type_str == "planar")
  {
    type = PLANAR;
    //ROS_WARN("Planar joints are deprecated in the URDF!\n");
  }
  else if (type_str == "floating")
  {
    type = FLOATING;
    //ROS_WARN("Floating joints are deprecated in the URDF!\n");
  }
  else if (type_str == "revolute")
    type = REVOLUTE;
  else if (type_str == "continuous")
    type = CONTINUOUS;
  else if (type_str == "prismatic")
    type = PRISMATIC;
  else if (type_str == "fixed")
    type = FIXED;
  else
  {
    ROS_ERROR("Joint '%s' has no known type '%s'", this->name.c_str(), type_str.c_str());
    return false;
  }

  // Get Joint Axis
  if (this->type != FLOATING && this->type != FIXED)
  {
    // axis
    TiXmlElement *axis_xml = config->FirstChildElement("axis");
    if (!axis_xml){
      //ROS_DEBUG("no axis elemement for Joint link '%s', defaulting to (1,0,0) axis", this->name.c_str());
      this->axis = Vector3(1.0, 0.0, 0.0);
    }
    else{
      if (axis_xml->Attribute("xyz")){
        try {
          this->axis.init(axis_xml->Attribute("xyz"));
        }
        catch (ParseError &e) {
          this->axis.clear();
          std::stringstream stm;
          stm << "Malformed axis element for joint ["<< this->name.c_str() << "]";
          throw ParseError(stm.str());
        }
      }
    }
  }

  // Get limit
  TiXmlElement *limit_xml = config->FirstChildElement("limit");
  if (limit_xml)
  {
    limits.reset(new JointLimits);
    if (!limits->initXml(limit_xml))
    {
      ROS_ERROR("Could not parse limit element for joint '%s'", this->name.c_str());
      limits.reset();
      return false;
    }
  }
  else if (this->type == REVOLUTE)
  {
    ROS_ERROR("Joint '%s' is of type REVOLUTE but it does not specify limits", this->name.c_str());
    return false;
  }
  else if (this->type == PRISMATIC)
  {
    //ROS_INFO("Joint '%s' is of type PRISMATIC without limits", this->name.c_str());
    limits.reset();
  }
  else if (this->type == CONTINUOUS)
  {
    limits.reset(new JointLimits);
    limits->lower = -1.0/0.0;
    limits->upper = 1.0/0.0;
  }
  else if (this->type == FIXED)
  {
    limits.reset(new JointLimits);
    limits->lower = -1.0/0.0;
    limits->upper = 1.0/0.0;
  }

  // Get safety
  TiXmlElement *safety_xml = config->FirstChildElement("safety_controller");
  if (safety_xml)
  {
    safety.reset(new JointSafety);
    if (!safety->initXml(safety_xml))
    {
      ROS_ERROR("Could not parse safety element for joint '%s'", this->name.c_str());
      safety.reset();
      return false;
    }
  }

  // Get calibration
  TiXmlElement *calibration_xml = config->FirstChildElement("calibration");
  if (calibration_xml)
  {
    calibration.reset(new JointCalibration);
    if (!calibration->initXml(calibration_xml))
    {
      ROS_ERROR("Could not parse calibration element for joint  '%s'", this->name.c_str());
      calibration.reset();
      return false;
    }
  }

  // Get Joint Mimic
  TiXmlElement *mimic_xml = config->FirstChildElement("mimic");
  if (mimic_xml)
  {
    mimic.reset(new JointMimic);
    if (!mimic->initXml(mimic_xml))
    {
      ROS_ERROR("Could not parse mimic element for joint  '%s'", this->name.c_str());
      mimic.reset();
      return false;
    }
  }

  // Get Dynamics
  TiXmlElement *prop_xml = config->FirstChildElement("dynamics");
  if (prop_xml)
  {
    dynamics.reset(new JointDynamics);
    if (!dynamics->initXml(prop_xml))
    {
      ROS_ERROR("Could not parse joint_dynamics element for joint '%s'", this->name.c_str());
      dynamics.reset();
      return false;
    }
  }

  return true;
}



}
