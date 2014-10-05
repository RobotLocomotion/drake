classdef Quadrotor < RigidBodyManipulator
  
  methods
    
    function obj = Quadrotor(sensor)
      if nargin<1, sensor=''; end
      options.floating = true;
      options.terrain = RigidBodyFlatTerrain();
      w = warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
      warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
      obj = obj@RigidBodyManipulator(getFullPathFromRelativePath('quadrotor.urdf'),options);
      warning(w);
      
      switch (sensor)
        case 'lidar'
          obj = addFrame(obj,RigidBodyFrame(findLinkInd(obj,'base_link'),[.35;0;0],zeros(3,1),'lidar_frame'));
          lidar = RigidBodyLidar('lidar',findFrameId(obj,'lidar_frame'),-.4,.4,40,10);
          lidar = enableLCMGL(lidar);
          obj = addSensor(obj,lidar);
        case 'kinect'
          obj = addFrame(obj,RigidBodyFrame(findLinkInd(obj,'base_link'),[.35;0;0],zeros(3,1),'kinect_frame'));
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
        shape = RigidBodyBox([.2+.8*rand(1,2) height],[xy;height/2],[0;0;randn]);
        shape.c = rand(3,1);
        obj = addShapeToBody(obj,'world',shape);
        obj = addContactShapeToBody(obj,'world',shape);
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
      obj = addShapeToBody(obj,'world',treeTrunk);
      obj = addContactShapeToBody(obj,'world',treeTrunk);
      treeLeaves = RigidBodyBox(1.5*[.2+.8*width_param height/4],...
          [xy;height + height/8],[0;0;yaw]);
      treeLeaves.c = [0,0.7,0];  % green
      obj = addShapeToBody(obj,'world',treeLeaves);
      obj = addContactShapeToBody(obj,'world',treeLeaves);
      obj = compile(obj);
    end    
    
    function obj = addBox(obj, lwh, xy, yaw)
      % Adds a single box (this is addTree but without the leaves) with specified length width height, xy
      % location, and yaw orientation.
      height = lwh(1,3);
      width_param = lwh(1,1:2);
      treeTrunk = RigidBodyBox([.2+.8*width_param height],...
          [xy;height/2+0.005],[0;0;yaw]);
      % treeTrunk.c = [83,53,10]/255;  % brown
      treeTrunk.c = [191,120,21]/255;  % brown
      obj = addShapeToBody(obj,'world',treeTrunk);
      obj = addContactShapeToBody(obj,'world',treeTrunk);
      obj = compile(obj);
    end    
    
    function obj = addFloatingBox(obj, lwh, xyz, yaw, color)
      if nargin < 5
        color = [.8,.8,.8]; % gray
      end
      % Adds a single box with specified length width height, x,y, and z
      % location, and yaw orientation.
      height = lwh(1,3);
      length = lwh(1,1);
      width = lwh(1,2);
      floatBox = RigidBodyBox([length width height],...
          [xyz(1,1) xyz(1,2) xyz(1,3)],[0;0;yaw]);
      floatBox.c = color;
      obj = addShapeToBody(obj,'world',floatBox);
      obj = addContactShapeToBody(obj,'world',floatBox);
      obj = compile(obj);
    end    
    
    function obj = addTable(r, lwh, xyz, dist)
      % This adds 5 Floating boxes given the size, location, tilt of
      % the Quadrant III leg, and shortest dist. from one leg to an
      % adjacent one
      
      %leg boxes
      r = addFloatingBox(r, lwh, [xyz(1,1),xyz(1,2),xyz(1,3)], 0);
      r = addFloatingBox(r, lwh, [xyz(1,1),xyz(1,2)+dist,xyz(1,3)], 0);
      r = addFloatingBox(r, lwh, [xyz(1,1)+dist,xyz(1,2),xyz(1,3)], 0);
      r = addFloatingBox(r, lwh, [xyz(1,1)+dist,xyz(1,2)+dist,xyz(1,3)], 0);
      %surface box
      obj = addFloatingBox(r, [dist+.3,dist+.3,0.1], [xyz(1,1)+dist/2,xyz(1,2)+dist/2, 2 * xyz(1,3)], 0);

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
  end
  
  
  methods (Static)
    
    function runOpenLoop
      r = Quadrotor('lidar');
      r = addTrees(r); 
      sys = TimeSteppingRigidBodyManipulator(r,.01);
      
      v = sys.constructVisualizer();

      x0 = [0;0;.5;zeros(9,1)];
      u0 = nominalThrust(r);
      
      sys = cascade(ConstantTrajectory(u0),sys);

%      sys = cascade(sys,v);
%      simulate(sys,[0 2],double(x0)+.1*randn(12,1));
      
      options.capture_lcm_channels = 'LCMGL';
      [ytraj,xtraj,lcmlog] = simulate(sys,[0 2],double(x0)+.1*randn(12,1),options);
      lcmlog
      v.playback(xtraj,struct('lcmlog',lcmlog));
%      figure(1); clf; fnplt(ytraj);
    end
  end
end