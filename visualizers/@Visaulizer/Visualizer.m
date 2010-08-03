classdef Visualizer
% Abstract class which provide interface and routines for visaulizers

  methods (Abstract=true)
    status = draw(obj,t,x,flag); 
  end

  properties 
    display_dt=0.05;
    playbackSpeed=1;
  end
  
end