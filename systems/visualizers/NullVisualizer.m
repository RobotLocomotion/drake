classdef NullVisualizer < Visualizer
% the visualizer that doesn't visualize (helpful for simulink simulation)
  
  methods
    function obj = NullVisualizer()  
      obj.display_dt = -1;
    end
    
    function status = draw(obj,t,x,flag)
      status = 0;
    end
  end
end
  
  