classdef XCubedVisualizer < Visualizer
% just for fun, plot a ball on a hill (who's negative gradient is the 
% \dot{x} = -x + x^3 dynamics)

methods 
  function obj=XCubedVisualizer()
    obj=obj@Visualizer(1);
  end
  
  function draw(obj,t,x)
    xs=linspace(-1.75,1.75,50);
    clf;
    plot(xs,obj.hill(xs),'LineWidth',2,'Color','k');
    hold on;
    r = .15;  % radius of the ball
    th = linspace(-pi,pi,50);
    patch(x+r*sin(th),obj.hill(x)+r+r*cos(th),[.7 .7 1]);
    axis([-1.75,1.75,-.5,.5]);
    axis equal
  end
  
end

methods (Static)
  function y=hill(x)
    y = .5*x.^2 - .25*x.^4;
    % NOTEST
  end
end

end
