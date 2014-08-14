classdef SCSDoublePendVisualizer < Visualizer
% Implements the draw function for the SCS Double Pendulum 

  methods
    function obj = SCSDoublePendVisualizer(dpobj)
      obj = obj@Visualizer(4);
      
      if (nargin>0)
        if (~isa(dpobj,'com.yobotics.exampleSimulations.doublePendulum.DoublePendulumRobot'))
          error('wrong type');
        end
        obj.l1 = dpobj.L1;
        obj.l2 = dpobj.L2;
      end
    end
    
    function draw(obj,t,x)
      x=[x(1)+pi;x(3);x(2);x(4)];
      
      % draw the acrobot
      persistent hFig L1r L1a L2r L2a;
      l1=obj.l1; l2=obj.l2;
  
      if (isempty(hFig))
        hFig = sfigure(32);
        set(hFig,'DoubleBuffer','on');
        av = pi/2*[1:.05:3];
        r = .04*min([l1 l2]);
        L1x = [r*cos(av) l1+r*cos(av+pi)];
        L1y = [r*sin(av) r*sin(av+pi)];
        L1r = (L1x.^2+L1y.^2).^.5;
        L1a = atan2(L1y,L1x);
        L2x = [r*cos(av) l2+r*cos(av+pi)];
        L2y = [r*sin(av) r*sin(av+pi)];
        L2r = (L2x.^2+L2y.^2).^.5;
        L2a = atan2(L2y,L2x);
      end
  
      sfigure(hFig);
      clf;
      
      patch(L1r.*sin(L1a+x(1)),-L1r.*cos(L1a+x(1)),0*L1a,'r');
      hold on
      patch(l1*sin(x(1))+L2r.*sin(L2a+x(1)+x(2)),-l1*cos(x(1))-L2r.*cos(L2a+x(1)+x(2)),1+0*L2a,'b');
      plot3(0,0,2,'k+');
      axis image
      view(0,90)
      set(gca,'XTick',[],'YTick',[])
      axis((l1+l2)*1.1*[-1 1 -1 1 -1 1000]);
      
      title(['t = ', num2str(t,'%.2f') ' sec']);
      drawnow;
    end
  end

  properties
    l1=1;    % length of link 1
    l2=2;    % length of link 2
  end
  
end

