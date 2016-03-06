classdef PendulumVisualizer < Visualizer
% Implements the draw function for the Pendulum 

% todo: use correct pendulum parameters (possibly acquire them via a
% constructor?)

  methods
    function obj = PendulumVisualizer()
      obj = obj@Visualizer(PendulumState);
    end
    
    function draw(obj,t,x)
      % Draw the pendulum.  
      persistent base a1 a2 ac1 ac2 raarm t0;

      if (isempty(a1))
        a1 = 0.75;  ac1 = 0.75; %415;
        av = pi*[0:.05:1];
        rb = .03; hb=.07;
        aw = .01;
        base = rb*[1 cos(av) -1 1; -hb/rb sin(av) -hb/rb -hb/rb]';
        arm = [aw*cos(av-pi/2) -a1+aw*cos(av+pi/2)
          aw*sin(av-pi/2) aw*sin(av+pi/2)]';
        raarm = [(arm(:,1).^2+arm(:,2).^2).^.5, atan2(arm(:,2),arm(:,1))];
      end
            
      patch(base(:,1), base(:,2),1+0*base(:,1),'b','FaceColor',[.3 .6 .4])
      patch(raarm(:,1).*sin(raarm(:,2)+x(1)-pi),...
        -raarm(:,1).*cos(raarm(:,2)+x(1)-pi), ...
        0*raarm(:,1),'r','FaceColor',[.9 .1 0])
      plot3(ac1*sin(x(1)), -ac1*cos(x(1)),1, 'ko',...
        'MarkerSize',14,'MarkerFaceColor','b')
      plot3(0,0,1.5,'k.')
      set(gca,'XTick',[],'YTick',[])
      
      axis image; axis([-1.0 1.0 -1.0 1.0]);
    end    
  end
  
end
