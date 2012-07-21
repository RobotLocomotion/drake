classdef HopperVisualizer < Visualizer
  
  % todo: make xcamera a state variable with a low-pass filter dynamics.
  
  properties
    l_1 = 0.5;       % distance from the foot to the com of the leg
    l_2 = 0.4;       % distance from the hip to the com of the body
  end
  
  methods
    function obj=HopperVisualizer(plant)
      typecheck(plant,'HopperPlant');
      obj = obj@Visualizer(plant.getOutputFrame);
    end
    
    function draw(obj,t,q)
      l_1 = obj.l_1;
      l_2 = obj.l_2;
      persistent hFig body leg foot xcamera;

      if (isempty(hFig) | ~ishandle(hFig))
        hFig = sfigure(25);
        set(hFig,'DoubleBuffer','on','NumberTitle','off',...
          'Name','Planar One-Leg Hopper Simulation');
        pos = get(hFig,'Position');
        set(hFig,'Position',[pos(1) pos(2) 800 300]);
        hold on;
        
        brick1 = [ 0.9 1.05 1.05 0.9 0.9; 0.15 0.15 0.0 0.0 0.15];
        brick2 = [-1 0; 0 1]*brick1;
        beam = [-1 -1 1 1; 0.15 0.175 0.175 0.15];
        comp = [0.25 0.25 0.025 0.025 -0.025 -0.025 -0.25 -0.25; 0.15 0.3 0.3 0.15 0.15 0.3 0.3 0.15];
        body = [brick1,brick2,beam,comp];
        body = body - [0;0.16]*ones(1,size(body,2));
        leg = [ -0.015 -0.04 -0.04 0.04 0.04 0.015 -0.015;
          -0.25 -0.25 0.525 0.525 -0.25 -0.25 -0.25 ];
        leg = leg - [0;0.16]*ones(1,size(leg,2));
        foot = [ 0.0 -0.03 -0.03 0.015 0.015 -0.025 -0.025 0.025 0.025 -0.015 -0.015 0.03 0.03 0.0;
          0.0  0.05  0.14 0.14 1.675 1.675 1.725 1.725 1.675 1.675 0.14 0.14 0.05 0.0 ];
        
        z = 0.02:0.01:0.79;
        last_t = 0;
        xcamera = 0;
      end
      
      hip = [ 0; q(5)];%+0.16];
      
      rot = [cos(q(3)) sin(q(3)); -sin(q(3)) cos(q(3))];
      hip = rot*hip + [q(1); q(2)];
      
      cop = [q(1) q(2)];
      
      for i=1:size(foot,2)
        Foot(:,i) = rot*foot(:,i) + [q(1); q(2)];
      end
      for i=1:size(leg,2)
        Leg(:,i) = rot*leg(:,i) + hip;
      end
      rot = [cos(q(4)) sin(q(4)); -sin(q(4)) cos(q(4))];
      for i=1:size(body,2)
        Body(:,i) = rot*body(:,i) + hip;
      end
      
      sfigure(hFig);
      clf;
      hold on;
      % Axes:
      
      xcamera = mean([hip(1),xcamera]);
      xmin = xcamera - 5;
      xmax = xcamera + 5;
      %  xmin = -2.5;%hip(1) - 2.5; %- 9.0; %- 2.5;
      %  xmax = 7.5;%hip(1) + 2.5; %+ 1.0; %+ 2.5;
      ymin = -0.25;
      ymax = 3.0;
      
      % draw the ground
      h = line([xmin;xmax],[0,0]);
      set(h,'Color',[0 0 0]);
      axis image;
      axis([xmin xmax ymin ymax]);
      
      line(Foot(1,:)',Foot(2,:)');
      line(Leg(1,:)',Leg(2,:)');
      line(Body(1,:)',Body(2,:)');
      c = 1 - 0.125*t;
      c = 0.75;
      fill(Foot(1,:)',Foot(2,:)',[c c c]);
      fill(Leg(1,:)',Leg(2,:)',[c c c]);
      fill(Body(1,:)',Body(2,:)',[c c c]);
    end
    
  end
end
