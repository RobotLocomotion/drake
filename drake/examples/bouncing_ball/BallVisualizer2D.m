classdef BallVisualizer2D < Visualizer
  
  methods 
    function obj = BallVisualizer2D(ballplant)
      typecheck(ballplant,'BallPlant2D');
      obj = obj@Visualizer(ballplant.getOutputFrame);  % number of inputs = 2
    end
    
    function draw(obj,t,y)
      hFig=sfigure(25);  % select figure 25 without forcing it to the front
      clf;
      axisAnnotation('ellipse',...               % draw circle
        [y(1), y(2), 0, 0] + obj.r*[-1,-1,2,2],...  % [x y w h]
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
