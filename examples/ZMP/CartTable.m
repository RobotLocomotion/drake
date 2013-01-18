classdef CartTable < TimeSteppingRigidBodyManipulator
  
  methods
    function obj = CartTable()
      obj = obj@TimeSteppingRigidBodyManipulator('CartTable.urdf',0.005,struct('floating',true));
    end
  end

  methods (Static = true)
    function run
      r = CartTable;
      xtraj = simulate(r,[0 7],[0;0;.12;0;.03;0;-.03;0;0;0;0;0;0;0;0;0]);
      v = r.constructVisualizer;
      v.playback(xtraj,struct('slider',true));
    end
  end
  
end