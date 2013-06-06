classdef ScopeVisualizer < Visualizer
% Simple utility to plot one particular index of a coordinate frame.    
% For instance, it's a useful way to test an LCMCoordinateFrame
% Todo: make this a lot better/prettier
  
  methods
    function obj = ScopeVisualizer(frame,ind)
      obj = obj@Visualizer(frame);
      obj.plot_index = ind;
      figure(1204); 
      clf; 
      hold on; 
      xlabel('t (sec)'); 
      ylabel(getCoordinateName(frame,ind));
    end
    
    function draw(obj,t,y)
      figure(1204);
      plot(t,y(obj.plot_index),'.');
    end
    
  end
  
  properties
    plot_index
  end
  
end