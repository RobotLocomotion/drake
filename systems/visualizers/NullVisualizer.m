classdef NullVisualizer < Visualizer
% the visualizer that doesn't visualize (helpful for simulink simulation)
  
  methods
    function obj = NullVisualizer(frame)  
      obj = obj@Visualizer(frame);
      obj.display_dt = -1;
    end
    
    function status = draw(obj,t,x,flag)
      status = 0;
    end
    
    function drawWrapper(obj,t,y)
      % intentionally blank
    end
    
    function playback(obj,xtraj,options)
      % intentionally blank
    end
    
  end
end
  
  