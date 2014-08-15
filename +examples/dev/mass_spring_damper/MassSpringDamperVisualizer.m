classdef MassSpringDamperVisualizer < Visualizer
% Implements the draw function for the Pendulum 

% todo: use correct pendulum parameters (possibly acquire them via a
% constructor?)

  methods
    function obj = MassSpringDamperVisualizer(r)
      obj = obj@Visualizer(r.getStateFrame);
    end
    
    function draw(obj,t,x)
      persistent hFig base wb lwheel rwheel spring dashpot;
      if (isempty(hFig))
        hFig = figure(25);
        set(hFig,'DoubleBuffer', 'on');
        
        theta = pi*[0:0.025:2];
        wb = .3; hb=.2;
        wheelr = 0.05;
        lwheel = [-wb/2 + wheelr*cos(theta); wheelr + wheelr*sin(theta)]';
        base = [wb*[1 -1 -1 1]; hb*[1 1 -1 -1]+hb+2*wheelr]';
        spring = [ 0,linspace(.2,.8,length(theta)),1; [0,.1*sin(8*theta),0]+.3 ];
      end
      
      figure(hFig); cla; hold on; view(0,90);
      
      x = x(1);
      % draw the cart
      patch(x(1)+base(:,1), base(:,2),0*base(:,1),'b','FaceColor',[.3 .6 .4])
      patch(x(1)+lwheel(:,1), lwheel(:,2), 0*lwheel(:,1),'k');
      patch(x(1)+wb+lwheel(:,1), lwheel(:,2), 0*lwheel(:,1),'k');
      
      % draw the floor
      line([-5,5],[0,0],'Color',.7*[1 1 1],'LineWidth',1.2);
      
      % draw the wall
      line([-1,-1],[0,1],'Color',.7*[1 1 1],'LineWidth',1.2);
      
      % draw the spring
      plot(-1+spring(1,:)*(x-wb+1),spring(2,:),'k');
      
      axis equal;
      axis([-1.5 1.5 -0.5 1]);
    end    
  end
  
end
