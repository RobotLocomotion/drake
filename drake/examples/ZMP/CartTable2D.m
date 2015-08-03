classdef CartTable2D < TimeSteppingRigidBodyManipulator
  
  methods
    function obj = CartTable2D()
      obj = obj@TimeSteppingRigidBodyManipulator('CartTable.urdf',0.005,struct('floating',true,'twoD',true,'terrain',RigidBodyFlatTerrain()));
      
      obj = addSensor(obj,FullStateFeedbackSensor());
      % Commented out pending rewrite of ContactForceTorqueSensor
      %ft_frame = RigidBodyFrame(findLinkId(obj,'base'),zeros(3,1),zeros(3,1),'ft_frame');
      %obj = addFrame(obj,ft_frame);
      %s = ContactForceTorqueSensor(obj,ft_frame);
%     %s.frame = CartTable2DFT;
      %obj = addSensor(obj,s);
      obj = compile(obj);
    end
    
%    function v = constructVisualizer(obj)
%%      v = MultiVisualizer({constructVisualizer@TimeSteppingRigidBodyManipulator(obj),obj.sensor{1}});
%      v = obj.sensor{1};
%    end
  end

  methods (Static = true)
    function run
      r = CartTable2D;
      x0 = Point(r.getStateFrame(), 0);
      x0.base_z = 0.12;
      x0.base_relative_pitch = 0.03;
      x0.cart_x = -0.0105;
      xtraj = simulate(r,[0 1],x0);
      v = r.constructVisualizer;
      v.playback(xtraj,struct('slider',true));
    end
  end
  
end
