classdef Quadrotor < RigidBodyManipulator
  
  properties
    m = .5;
    I = diag([0.0023,0.0023,0.004]);
  end
    
  methods
    
    function obj = Quadrotor(sensor,floating_base_type)
      if nargin<1, sensor=''; end
      if (nargin<2) 
        options.floating = true; 
      else
        options.floating = floating_base_type;
      end
      options.terrain = RigidBodyFlatTerrain();
      w = warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
      warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
      obj = obj@RigidBodyManipulator(getFullPathFromRelativePath('quadrotor.urdf'),options);
      warning(w);
      
      switch (sensor)
        case 'lidar'
          obj = addFrame(obj,RigidBodyFrame(findLinkId(obj,'base_link'),[.35;0;0],zeros(3,1),'lidar_frame'));
          lidar = RigidBodyLidar('lidar',findFrameId(obj,'lidar_frame'),-.4,.4,40,10);
          lidar = enableLCMGL(lidar);
          obj = addSensor(obj,lidar);
        case 'kinect'
          obj = addFrame(obj,RigidBodyFrame(findLinkId(obj,'base_link'),[.35;0;0],zeros(3,1),'kinect_frame'));
          kinect = RigidBodyDepthSensor('kinect',findFrameId(obj,'kinect_frame'),-.4,.4,12,-.5,.5,30,10);
          kinect = enableLCMGL(kinect);
          obj = addSensor(obj,kinect);
      end
      obj = addSensor(obj,FullStateFeedbackSensor);
      obj = compile(obj);
    end
   
    function I = getInertia(obj)
      I = obj.body(2).inertia;
    end
    
    function nW = getNumDisturbances(obj)
        nW = 3;
    end
    
    function u0 = nominalThrust(obj)
      % each propellor commands -mg/4
      u0 = Point(getInputFrame(obj),getMass(obj)*norm(getGravity(obj))*ones(4,1)/4);
    end
    
    function obj = addObstacles(obj,number_of_obstacles)
      if nargin<2, number_of_obstacles = randi(10); end
      
      for i=1:number_of_obstacles
        xy = randn(2,1);
        while(norm(xy)<1), xy = randn(2,1); end
        height = .5+rand;
        geometry = RigidBodyBox([.2+.8*rand(1,2) height],[xy;height/2],[0;0;randn]);
        geometry.c = rand(3,1);
        obj = addGeometryToBody(obj,'world',geometry);
      end
      
      obj = compile(obj);
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
      obj = addGeometryToBody(obj,'world',treeTrunk, 'treeTrunk');
      treeLeaves = RigidBodyBox(1.5*[.2+.8*width_param height/4],...
          [xy;height + height/8],[0;0;yaw]);
      treeLeaves.c = [0,0.7,0];  % green
      obj = addGeometryToBody(obj,'world',treeLeaves, 'treeLeaves');
      obj = compile(obj);
    end    
    
    function traj_opt = addPlanVisualizer(obj,traj_opt)
      % spew out an lcmgl visualization of the trajectory.  intended to be
      % used as a callback (fake objective) in the direct trajectory
      % optimization classes

      if ~checkDependency('lcmgl')
        warning('lcmgl dependency is missing.  skipping visualization'); 
        return;
      end
      lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'QuadrotorPlan');
      
      typecheck(traj_opt,'DirectTrajectoryOptimization');

      traj_opt = traj_opt.addDisplayFunction(@(x)visualizePlan(x,lcmgl),traj_opt.x_inds(1:3,:));
      
      function visualizePlan(x,lcmgl)
        lcmgl.glColor3f(1, 0, 0);
        lcmgl.glPointSize(3);
        lcmgl.points(x(1,:),x(2,:),x(3,:));
        lcmgl.glColor3f(.5, .5, 1);
        lcmgl.plot3(x(1,:),x(2,:),x(3,:));
        lcmgl.switchBuffers;
      end
    end
    
    function [f,df,d2f] = dynamics(obj,t,x,u)
        q = x(1:6);
        qd = x(7:12);
        qdd = obj.sodynamics(t,q,qd,u);
        
        f = [qd; qdd];
        
        if (nargout>1)
            [df,d2f] = dynamicsGradients(obj,t,x,u,nargout-1);
        end
    end
    
    function [f,df,d2f] = dynamics_w(obj,t,x,u,w)
        q = x(1:6);
        qd = x(7:12);
        qdd = obj.sodynamics(t,q,qd,u) + [w./obj.m; 0; 0; 0];
        
        f = [qd; qdd];
        
        if (nargout>1)
            [df,d2f] = dynamicsGradients_w(obj,t,x,u,w,nargout-1);
        end
    end
    
    function qdd = sodynamics(obj,t,q,qd,u)
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
        
        
        xyz_ddot = (1/m)*([0;0;-m*g] + R*[0;0;F1+F2+F3+F4]);
        
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
  end
  
  
  methods (Static)
    
    function runOpenLoop
      r = Quadrotor('lidar','quat');
      r = addTrees(r); 
      sys = TimeSteppingRigidBodyManipulator(r,.01);
      
      v = sys.constructVisualizer();

      x0 = [getRandomConfiguration(r);zeros(6,1)];
      x0(3) = .5;
      u0 = nominalThrust(r);
      
      sys = cascade(ConstantTrajectory(u0),sys);

%      sys = cascade(sys,v);
%      simulate(sys,[0 2],double(x0)+.1*randn(12,1));
      
      options.capture_lcm_channels = 'LCMGL';
      [ytraj,xtraj,lcmlog] = simulate(sys,[0 2],double(x0),options);
      lcmlog
      v.playback(xtraj,struct('lcmlog',lcmlog));
%      figure(1); clf; fnplt(ytraj);
    end
  end
end
