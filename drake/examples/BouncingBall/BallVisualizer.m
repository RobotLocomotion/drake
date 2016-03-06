classdef BallVisualizer < Visualizer
  
  methods 
    function obj = BallVisualizer(fr)
      obj = obj@Visualizer(fr);  % number of inputs = 1
    end
    
    function draw(obj,t,y)
      hFig=sfigure(25);  % select figure 25 without forcing it to the front
      clf;
      axisAnnotation('ellipse',...               % draw circle
        [0; y(1); 0; 0] + obj.r*[-1;-1;2;2],...  % [x y w h]
        'FaceColor','r');                        % make it red
      line([-5,5]*obj.r,[0,0],'Color','k','LineWidth',1.5);
      axis equal;
      axis(obj.r*[-5 5 -.5 9.5]);
    end
  end
  
  properties
    r = 1;  % radius of the ball
  end
end
