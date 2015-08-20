classdef IRB140Input < SingletonCoordinateFrame
  % irb140 input coordinate frame
  methods
    function obj=IRB140Input(r)
      typecheck(r,{'TimeSteppingRigidBodyManipulator','RigidBodyManipulator'});

      manipInputFrame = r.getManipulator().getInputFrame();
      if (r.hands > 0)
        manipInputFrame = manipInputFrame.getFrameByNum(1);
      end
      input_names = manipInputFrame.getCoordinateNames();
      input_names = regexprep(input_names,'_motor',''); % remove motor suffix     
      
      obj = obj@SingletonCoordinateFrame('IRB140Input',length(input_names),'x',input_names);
    end
  end
end
