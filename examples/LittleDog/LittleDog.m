classdef LittleDog < RigidBodyManipulator %& LeggedRobot
  
  methods
    function obj = LittleDog
      options.floating = true;
      options.terrain = RigidBodyFlatTerrain();
      w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
      obj = obj@RigidBodyManipulator('LittleDog.urdf',options);
      warning(w);
      
%      obj = addFoot(obj, 'front_left', 'front_left_foot_center');
%      obj = addFoot(obj, 'front_right', 'front_right_foot_center');
%      obj = addFoot(obj, 'back_left', 'back_left_foot_center');
%      obj = addFoot(obj, 'back_right', 'back_right_foot_center');
    end
    
    function x0 = home(obj)
      hip_roll = .1;
      hip_pitch = 1;
      knee = 1.55;
      x0 = Point(getStateFrame(obj));
      x0.front_right_hip_roll = -hip_roll;
      x0.front_right_hip_pitch = hip_pitch;
      x0.front_right_knee = -knee;
      x0.front_left_hip_roll = hip_roll;
      x0.front_left_hip_pitch = hip_pitch;
      x0.front_left_knee = -knee;
      x0.back_right_hip_roll = -hip_roll;
      x0.back_right_hip_pitch = -hip_pitch;
      x0.back_right_knee = knee;
      x0.back_left_hip_roll = hip_roll;
      x0.back_left_hip_pitch = -hip_pitch;
      x0.back_left_knee = knee;
      x0 = resolveConstraints(obj,x0);
    end
    
    function xstar = defaultFixedPoint(obj)
      xstar = home(obj); %findFixedPoint(obj,home(obj));
    end
    
    function plan = planFootsteps(obj)
      error('not implemented yet')
    end
    
  end
  
  methods (Static=true)
    function runPassive
      r = LittleDog;
      sys = TimeSteppingRigidBodyManipulator(r,0.01);
      x0 = home(r);
      
      v = constructVisualizer(sys);
      xtraj = simulate(sys,[0 2],double(x0));
      playback(v,xtraj);
    end
    
    function runSagittalPassive
      options.twoD = true;
      options.view = 'right';
      options.floating = true;
      options.terrain = RigidBodyFlatTerrain();
      sys = TimeSteppingRigidBodyManipulator('LittleDogTwoLegs.urdf',0.005,options);
      
      v = constructVisualizer(sys);
      xtraj = simulate(sys,[0 2]);
      playback(v,xtraj);
    end
    
    function runPDhome
      r = LittleDog;
      sys = TimeSteppingRigidBodyManipulator(r,0.0015);
      v = constructVisualizer(sys);
      
      % construct PD control 
      Kp = 2*eye(12);
      Kd = diag([0.5; 0.5; 0.16; 0.5; 0.5; 0.16; 0.5; 0.5; 0.16; 0.5; 0.5; 0.16]);
      sys = pdcontrol(sys,Kp,Kd);
      
      x0 = home(r);
      qa0 = x0(getActuatedJoints(r));

      % send in constant position reference
      sys = cascade(setOutputFrame(ConstOrPassthroughSystem(qa0),getInputFrame(sys)),sys);

      if (1)
        sys = cascade(sys,v);
        simulate(sys,[0 4],double(x0));
      else
        xtraj = simulate(sys,[0 4],double(x0));
        playback(v,xtraj);
      end
    end
  end
end