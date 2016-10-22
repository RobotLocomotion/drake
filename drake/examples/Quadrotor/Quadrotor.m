classdef Quadrotor < RigidBodyManipulator
  
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
      lcmgl = drake.matlab.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'QuadrotorPlan');
      
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
