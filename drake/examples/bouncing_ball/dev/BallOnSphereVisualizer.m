classdef BallOnSphereVisualizer < Visualizer
  
  methods 
    function obj = BallOnSphereVisualizer(b)
      typecheck(b,'BallOnSpherePlant');
      obj = obj@Visualizer(b.getOutputFrame);  % number of inputs = 2
    end
    
    function draw(obj,t,y)
      hFig=sfigure(25);  % select figure 25 without forcing it to the front
      clf;
      plot(y(1),y(2),'r.');
      hold on;
      axisAnnotation('ellipse',...               % draw sphere
        [-1;-1;2;2],...  % [x y w h]
        'LineWidth',2);                        
      axis equal;
      axis([-2 2 -2 2]);
    end
  end
  
end
