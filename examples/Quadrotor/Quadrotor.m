classdef Quadrotor < RigidBodyManipulator
  
  methods
    
    function obj = Quadrotor()
      options.floating = true;
      options.terrain = RigidBodyFlatTerrain();
      w = warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
      warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
      obj = obj@RigidBodyManipulator('quadrotor.urdf',options);
      warning(w);
      
      obj = addFrame(obj,RigidBodyFrame(findLinkInd(obj,'base_link'),[.35;0;0],zeros(3,1),'lidar_frame'));
      lidar = RigidBodyLidar('lidar',findFrameId(obj,'lidar_frame'),0,0,1,10);
      obj = addSensor(obj,FullStateFeedbackSensor);
      obj = addSensor(obj,lidar);
      obj = compile(obj);
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
        s = RigidBodyBox([.2+.8*rand(1,2) height],[xy;height/2],[0;0;randn]);
        s.c = rand(3,1);
        obj = addShapeToBody(obj,'world',s);
        obj = addContactShapeToBody(obj,'world',s);
      end
      
      obj = compile(obj);
    end
  end
  
  
  methods (Static)
    function runOpenLoop
      r = Quadrotor();
      r = addObstacles(r); 
      sys = TimeSteppingRigidBodyManipulator(r,.01);
      
      v = r.constructVisualizer();

      x0 = [0;0;.5;zeros(9,1)];
      u0 = nominalThrust(r);
      
      sys = cascade(ConstantTrajectory(u0),sys);

      sys = cascade(sys,v);
      simulate(sys,[0 2],double(x0)+.1*randn(12,1));
      
%      [ytraj,xtraj] = simulate(sys,[0 2],double(x0)+.1*randn(12,1));
%      v.playback(xtraj);
%      figure(1); clf; fnplt(ytraj);
    end
  end
end