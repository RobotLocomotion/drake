classdef FullStateFeedbackSensor < RigidBodySensor
  
  methods
    function y = output(obj,manip,t,x,u)
      y=x;
    end
    function fr = constructFrame(obj,manip)
      fr = manip.getStateFrame;
    end
    function tf = isDirectFeedthrough(obj)
      tf=false;
    end
  end
end