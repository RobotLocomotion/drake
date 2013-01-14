classdef FullStateFeedbackSensor < RigidBodySensor
  
  properties
    frame
  end
  
  methods
    function obj = FullStateFeedbackSensor(manip)
      typecheck(manip,'RigidBodyManipulator');
      obj.frame = manip.getStateFrame();
    end
    function y = output(obj,t,x,u)
      y=x;
    end
    function fr = getFrame(obj)
      fr = obj.frame;
    end
  end
end