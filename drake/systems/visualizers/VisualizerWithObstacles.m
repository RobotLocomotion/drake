classdef VisualizerWithObstacles < Visualizer
% Visualizer that also draws obstacles.

  methods (Abstract=true)
    draw(obj,t,x); % draw function interface
  end

  methods 
    function obj=VisualizerWithObstacles(num_u, obstacle_field)
      obj=obj@Visualizer(num_u);
      obj.obstacle_field = obstacle_field;
    end
    
    function obj = SetObstacleField(obj, obstacle_field)
      obj.obstacle_field = obstacle_field;
    end
    
  end
  
  properties
    obstacle_field;
  end
  
end
