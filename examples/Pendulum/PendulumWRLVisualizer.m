classdef PendulumWRLVisualizer < Visualizer
% Implements the draw function for the Pendulum 

% todo: use correct pendulum parameters (possibly acquire them via a
% constructor?)

  methods
    function obj = PendulumWRLVisualizer()
      obj = obj@Visualizer(PendulumState);
      obj.wrl = vrworld('Pendulum.wrl');
      open(obj.wrl);
      view(obj.wrl);
    end
    
    function draw(obj,t,x)
      % Draw the pendulum.  
      obj.wrl.theta.rotation=[0 1 0 x(1)];
    end    
  end
  
  properties
    wrl
  end
  
end
