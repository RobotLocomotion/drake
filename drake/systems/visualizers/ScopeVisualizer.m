classdef ScopeVisualizer < Visualizer
% Simple utility to plot one particular index of a coordinate frame.    
% For instance, it's a useful way to test an LCMCoordinateFrame
% Todo: make this a lot better/prettier
  
  methods
    function obj = ScopeVisualizer(frame)
      obj = obj@Visualizer(frame);
      figure(1204); 
      clf; 
      numu = getNumInputs(obj);
      cs = regexprep(getCoordinateNames(frame),'_',' ');
      obj.plotrows = ceil(sqrt(numu)); obj.plotcols=ceil(numu/obj.plotrows);
      for i=1:getNumInputs(obj)
        subplot(obj.plotrows,obj.plotcols,i); hold on;
        xlabel('t (sec)');
        ylabel(cs{i});
      end
    end
    
    function draw(obj,t,y)
      figure(1204); 
      for i=1:getNumInputs(obj)
        subplot(obj.plotrows,obj.plotcols,i); 
        plot(t,y(i),'.','MarkerSize',1.5);
      end
    end
    
  end
  
  properties
    plotrows
    plotcols
  end
  
end