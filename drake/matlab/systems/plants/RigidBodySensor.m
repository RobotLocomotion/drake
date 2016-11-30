classdef RigidBodySensor < RigidBodyElement
% this is an interface class for sensors that are composed to form the
% output output of a RigidBodyManipulator
%

% note that an implementation alternative would be to have sensors act as a
% separate dynamical system, but this seems more natural since many sensors
% need to know both x and u, and will not have any state.  We might want to
% revisit this decision at a later time.
  
  methods (Abstract=true)
    y = output(obj,manip,t,x,u);
    fr = constructFrame(obj,manip);
    tf = isDirectFeedthrough(obj);
  end
  
  methods 
    function obj = compile(obj,manip)
      obj.coordinate_frame = constructFrame(obj,manip);
    end
    
    function obj = updateBodyIndices(obj,map_from_old_to_new)
      % intentionally do nothing
    end
    
    function obj = updateBodyCoordinates(obj,body_ind,T_old_body_to_new_body)
      error('probably need to implement this (see changeRootLink)');
    end
  end
  
  properties
    name;  % name of this sensor as a string
    coordinate_frame;  % the output frame associated with this sensor
  end
end
    