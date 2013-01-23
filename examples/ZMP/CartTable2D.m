classdef CartTable2D < TimeSteppingRigidBodyManipulator
  
  methods
    function obj = CartTable2D()
      obj = obj@TimeSteppingRigidBodyManipulator('CartTable.urdf',0.005,struct('floating',true,'twoD',true));
      
      obj = addSensor(obj,FullStateFeedbackSensor());
      s = ContactForceTorqueSensor(obj,'base+base_part2+base_part3',[0;0],0);
      s.frame = CartTable2DFT;
      obj = addSensor(obj,s);
      obj = compile(obj);
    end
    
    function v = constructVisualizer(obj)
%      v = MultiVisualizer({constructVisualizer@TimeSteppingRigidBodyManipulator(obj),obj.sensor{1}});
      v = obj.sensor{1};
    end
  end

  methods (Static = true)
    function run
      r = CartTable2D;
      xtraj = simulate(r,[0 1],[0;.12;.03;-.0105;0;0;0;0]);
      v = r.constructVisualizer;
      v.playback(xtraj,struct('slider',true));
    end
  end
  
end