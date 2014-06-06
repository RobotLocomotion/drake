classdef Quadrotor < RigidBodyManipulator
  
  methods
    
    function obj = Quadrotor()
      options.floating = true;
      obj = obj@RigidBodyManipulator('quadrotor.urdf',options);
    end
   
    function u0 = nominalThrust(obj)
      % each propellor commands -mg/4
      u0 = getMass(obj)*norm(obj.gravity)*ones(4,1)/4;
    end
  end
  
  
  methods (Static)
    function runOpenLoop
      r = Quadrotor();
      v = r.constructVisualizer();

      x0 = [0;0;.5;zeros(9,1)];
      u0 = nominalThrust(r);
      
      sys = cascade(setOutputFrame(ConstantTrajectory(u0),getInputFrame(r)),r);

      xtraj = simulate(sys,[0 1],double(x0)+.1*randn(12,1));
      v.playback(xtraj);
    end
  end
end