classdef TimeSteppingRigidBodySensor 
  % analogous to RigidBodySensor, but for TimeStepping manipulators
  
  methods (Abstract=true)
    y = output(obj,tsmanip,manip,t,x,u);
    fr = getFrame(obj,tsmanip);
    tf = isDirectFeedthrough(obj);
  end
  
  methods 
    function obj = compile(obj,tsmanip,manip)
      % intentionally do nothing here, but can be overloaded if this
      % functionality is needed
    end
  end
  
end