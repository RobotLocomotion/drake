classdef Visualizer
% Abstract class which provide interface and routines for visaulizers

  methods (Abstract=true)
    status = draw(obj,t,x,flag); % draw function interface
  end

  properties 
    display_dt=0.05;  % requested time between display frames
    playbackSpeed=1;  % 1=realtime
    bDrawAxes=false;  % when making movies true=gcf,false=gca
  end
  
end