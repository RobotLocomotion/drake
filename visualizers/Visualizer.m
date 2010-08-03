classdef Visualizer
  
  properties (SetAccess=private)
    displayDT=0.05;
    playbackSpeed=1;
  end
  
  methods (Abstract=true)
    status = draw(obj,t,x,flag); 
  end

end