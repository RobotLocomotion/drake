classdef Pendulum < Robot

  methods 
    function obj = Pendulum
      obj.dynamics = PendulumDynamics;
      obj.control = ZeroControl(1);
      obj.visualizer = PendulumVisualizer;
    end
  end
end