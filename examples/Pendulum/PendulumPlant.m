classdef PendulumPlant < SecondOrderSystem
% Defines the dynamics for the Pendulum.
  
  properties
    m = 1;   % kg
    l = .5;  % m
    b = 0.1; % kg m^2 /s
    lc = .5; % m
    I = .25; %m*l^2; % kg*m^2
    g = 9.8; % m/s^2
  end
  
  methods
    function obj = PendulumPlant()
      % Construct a new PendulumPlant
      obj = obj@SecondOrderSystem(1,1,true);

      % setup input
      input_frame = CoordinateFrame.getSingleton('Pendulum:u',1);
      input_frame.setCoordinateNames({'tau'});
      obj = setInputFrame(obj,input_frame);
      torque_limit = 3;
      obj = setInputLimits(obj,-torque_limit,torque_limit);
      
      % setup state
      state_frame = CoordinateFrame.getSingleton('Pendulum:x',2);
      state_frame.setCoordinateNames({'\theta','\dot\theta'});
      state_frame.setAngleFlags([true;false]);
      obj = setStateFrame(obj,state_frame);
      
      % setup output
      obj = setOutputFrame(obj,state_frame);  % output is same as state
    end
    
    function qdd = sodynamics(obj,t,q,qd,u)
      % Implement the second-order dynamics
      qdd = (u - obj.m*obj.g*obj.lc*sin(q) - obj.b*qd)/obj.I;
    end
    
    function [f,df,d2f,d3f]=dynamics(obj,t,x,u)
      f=dynamics@SecondOrderSystem(obj,t,x,u);
      if (nargout>1)
        [df,d2f,d3f]= dynamicsGradients(obj,t,x,u,nargout-1);
      end
    end
    
    function x = getInitialState(obj)
      % Start me anywhere!
      x = randn(2,1);
    end
  end  

  methods(Static)
    function run()  % runs the passive system
      pd = PendulumPlant;
      pv = PendulumVisualizer;
      traj = simulate(pd,[0 5],randn(2,1));
      playback(pv,traj);
    end
  end
end
