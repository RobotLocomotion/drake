classdef RigidBodyActuator < RigidBodyElement
% trivial definition of the actuators allows us to define the mapping from 
% input to joint torques  
 
% todo: consider making the input much more rich (e.g. with the types of
% actuator models that Jessy is building for Atrias), and act like the
% complement to the RigidBodySensors

  properties
    name
    joint
    reduction=1
  end

  methods    
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      obj.joint = map_from_old_to_new(obj.joint);
    end
  end
end
