classdef CartPoleVisualizer < Visualizer
  
  properties
    l=0.5;
  end
  
  methods
    function obj = CartPoleVisualizer(plant)
      typecheck(plant,'CartPolePlant');
      obj = obj@Visualizer(plant.getOutputFrame);
      obj.l = plant.l;
    end
    
    function draw(obj,t,x)
      l=obj.l;
      persistent hFig base a1 raarm wb lwheel rwheel;
      if (isempty(hFig))
        hFig = sfigure(25);
        set(hFig,'DoubleBuffer', 'on');
        
        a1 = l;
        av = pi*[0:.05:1];
        theta = pi*[0:0.05:2];
        wb = .3; hb=.15;
        aw = .01;
        wheelr = 0.05;
        lwheel = [-wb/2 + wheelr*cos(theta); -hb-wheelr + wheelr*sin(theta)]';
        base = [wb*[1 -1 -1 1]; hb*[1 1 -1 -1]]';
        arm = [aw*cos(av-pi/2) -a1+aw*cos(av+pi/2)
          aw*sin(av-pi/2) aw*sin(av+pi/2)]';
        raarm = [(arm(:,1).^2+arm(:,2).^2).^.5, atan2(arm(:,2),arm(:,1))];
      end
      
      sfigure(hFig); cla; hold on; view(0,90);
      patch(x(1)+base(:,1), base(:,2),0*base(:,1),'b','FaceColor',[.3 .6 .4])
      patch(x(1)+lwheel(:,1), lwheel(:,2), 0*lwheel(:,1),'k');
      patch(x(1)+wb+lwheel(:,1), lwheel(:,2), 0*lwheel(:,1),'k');
      patch(x(1)+raarm(:,1).*sin(raarm(:,2)+x(2)-pi),-raarm(:,1).*cos(raarm(:,2)+x(2)-pi), 1+0*raarm(:,1),'r','FaceColor',[.9 .1 0])
      plot3(x(1)+l*sin(x(2)), -l*cos(x(2)),1, 'ko',...
        'MarkerSize',14,'MarkerFaceColor','b')
      plot3(x(1),0,1.5,'k.')
      set(gca,'XTick',[],'YTick',[])
      
      axis image;
      axis([-2.5 2.5 -2.5*l 2.5*l]);
    end
  end
end
