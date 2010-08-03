classdef Visualizer
% Abstract class which provide interface and routines for visaulizers

  methods (Abstract=true)
    status = draw(obj,t,x,flag); % draw function interface
  end

  properties 
    display_dt=0.05;  % requested time between display frames
    playback_speed=1;  % 1=realtime
    draw_axes=false;  % when making movies true=gcf,false=gca
  end
  
end