classdef QuadWindPlant < SecondOrderSystem
  % Modified from QuadPlantPenn
  
  % Modified from D. Mellinger, N. Michael, and V. Kumar,
  % "Trajectory generation and control for precise aggressive maneuvers with quadrotors",
  %  In Proceedings of the 12th International Symposium on Experimental Robotics (ISER 2010), 2010.
  
  methods
    function obj = QuadWindPlant()
      obj = obj@SecondOrderSystem(6,4,1);
      obj = setStateFrame(obj,CoordinateFrame('QuadState',12,'x',{'x','y','z','roll','pitch','yaw','xdot','ydot','zdot','rolldot','pitchdot','yawdot'}));
      obj = obj.setOutputFrame(obj.getStateFrame);
    end
    
    function m = getMass(obj)
      m = obj.m;
    end
    
    function I = getInertia(obj)
      I = obj.I;
    end
    
    function u0 = nominalThrust(obj)
      % each propellor commands -mg/4
      u0 = Point(getInputFrame(obj),getMass(obj)*norm(obj.gravity)*ones(4,1)/4);
    end
    
    %taken from rigid body manipulator
    function g = getGravity(obj)
      g = r.gravity
    end
    
    function [qdd, df] = sodynamics(obj,t,q,qd,u)
      % States
      % x
      % y
      % z
      % phi (roll)
      % theta (pitch)
      % psi (yaw)
      % xdot
      % ydot
      % zdot
      % phidot
      % thetadot
      % psidot
      
      if (nargout>1)
        [df]= dynamicsGradients(obj,t,q,qd,u,nargout-1);
      end
      
      
      % Parameters
      m = obj.m;
      I = obj.I;
      invI = diag(1./[0.0023,0.0023,0.004]);
      g = 9.81;
      L = 0.1750;
      
      % states
      x = [q;qd];
      
      phi = x(4);
      theta = x(5);
      psi = x(6);
      
      phidot = x(10);
      thetadot = x(11);
      psidot = x(12);
      
      w1 = u(1);
      w2 = u(2);
      w3 = u(3);
      w4 = u(4);
      
      % Rotation matrix from body to world frames
      R = rpy2rotmat([phi;theta;psi]);
      
      kf = 1; % 6.11*10^-8;
      
      F1 = kf*w1;
      F2 = kf*w2;
      F3 = kf*w3;
      F4 = kf*w4;
      
      km = 0.0245;
      
      M1 = km*w1;
      M2 = km*w2;
      M3 = km*w3;
      M4 = km*w4;
      
      xquad = x(1);
      yquad = x(2);
      zquad = x(3);
      quadpos = [xquad;yquad;zquad];
      
      
      [windout,dquadinwind] = obj.quadwind(quadpos);
      
      if (nargout>1)
        df = df + dquadinwind;
      end
      
      xyz_ddot = (1/m)*([0;0;-m*g] + R*[0;0;F1+F2+F3+F4] + windout); % call to wind field in dynamics
      
      pqr = rpydot2angularvel([phi;theta;psi],[phidot;thetadot;psidot]);
      pqr = R'*pqr;
      
      pqr_dot = invI*([L*(F2-F4);L*(F3-F1);(M1-M2+M3-M4)] - cross(pqr,I*pqr));
      
      % Now, convert pqr_dot to rpy_ddot
      [Phi, dPhi] = angularvel2rpydotMatrix([phi;theta;psi]);
      
      Rdot =  [ 0, sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta),   cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta); ...
        0, cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi), - cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta); ...
        0,                              cos(phi)*cos(theta),                               -cos(theta)*sin(phi)]*phidot + ...
        [ -cos(psi)*sin(theta), cos(psi)*cos(theta)*sin(phi), cos(phi)*cos(psi)*cos(theta); ...
        -sin(psi)*sin(theta), cos(theta)*sin(phi)*sin(psi), cos(phi)*cos(theta)*sin(psi); ...
        -cos(theta),         -sin(phi)*sin(theta),         -cos(phi)*sin(theta)]*thetadot + ...
        [ -cos(theta)*sin(psi), - cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta), cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta); ...
        cos(psi)*cos(theta),   cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta); ...
        0,                                                  0,                                                0]*psidot;
      
      rpy_ddot = Phi*R*pqr_dot + reshape((dPhi*[phidot;thetadot;psidot]),3,3)*R*pqr + ...
        Phi*Rdot*pqr;
      
      % xdot = [x(7:12);xyz_ddot;rpy_ddot];
      
      
      
      qdd = [xyz_ddot;rpy_ddot];
    end
    
    function [wind,dquadinwind] = quadwind(obj,quadpos)
      % quadpos is [xquad;yquad;zquad]
      
      xquad = quadpos(1);
      yquad = quadpos(2);
      zquad = quadpos(3);
      
      xwind = 0;
      ywind = 10*zquad; 
      zwind = 0;
      
      
      wind = [xwind;ywind;zwind];
      dquadinwind = sparse(6,17);
      dquadinwind(2,4) = 10/obj.m; 
      
    end
    
    
    function y = output(obj,t,x,u)
      y = x;
    end
    
    function x = getInitialState(obj)
      x = zeros(12,1);
    end
    
    function obj = addTrees(obj,number_of_obstacles)
      % Adds a random forest of trees
      if nargin<2, number_of_obstacles = 5*(randi(5)+2); end
      for i=1:number_of_obstacles
        % Populates an area of the forest
        xy = [20,0;0,12]*(rand(2,1) - [0.5;0]);
        % Creates a clear path through the middle of the forest
        while(norm(xy)<1 || (xy(1,1)<=1.5 && xy(1,1)>=-1.5)), xy = randn(2,1); end
        height = 1+rand;
        width_param = rand(1,2);
        yaw = randn;
        obj = obj.addTree([width_param height],xy,yaw);
      end
      obj = compile(obj);
    end
    
    function obj = addTree(obj, lwh, xy, yaw)
      % Adds a single tree with specified length width height, xy
      % location, and yaw orientation.
      height = lwh(1,3);
      width_param = lwh(1,1:2);
      treeTrunk = RigidBodyBox([.2+.8*width_param height],...
        [xy;height/2],[0;0;yaw]);
      treeTrunk.c = [83,53,10]/255;  % brown
      obj = addGeometryToBody(obj,'world',treeTrunk);
      treeLeaves = RigidBodyBox(1.5*[.2+.8*width_param height/4],...
        [xy;height + height/8],[0;0;yaw]);
      treeLeaves.c = [0,0.7,0];  % green
      obj = addGeometryToBody(obj,'world',treeLeaves);
      obj = compile(obj);
    end
    
  end
  properties
    m = .5;
    I = diag([0.0023,0.0023,0.004]);
    
    % defining gravity here (too simple?)
    gravity = [0; 0; -9.8100]
  end
  
end



