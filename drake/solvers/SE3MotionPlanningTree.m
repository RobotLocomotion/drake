classdef SE3MotionPlanningTree < CompositeVertexArrayTree
  properties
    rbm
    min_distance = 0.2;
  end

  methods
    function obj = SE3MotionPlanningTree()
      T_R3 = R3MotionPlanningTree();
      T_SO3 = SO3MotionPlanningTree();
      obj = obj@CompositeVertexArrayTree({T_R3, T_SO3},{'R3', 'SO3'});
      obj = obj.setLCMGL('SE3MotionPlanningTree',[0, 0, 0]);
      urdf = fullfile(getDrakePath, 'systems', 'plants', 'test', 'FallingBrick.urdf');
      options.floating = true;
      obj.rbm = RigidBodyManipulator(urdf, options);
      obj.rbm = obj.rbm.removeCollisionGroupsExcept({});
      % TODO: Switch to something like the below.
      %obj.rbm = RigidBodyManipulator();
      %obj.rbm.name{1} = 'robot';
      %body = RigidBody();
      %body = body.setInertial(1, zeros(3,1), eye(3));
      %body.linkname = 'body';
      %body.robotnum = 1;
      %obj.rbm = obj.rbm.addLink(body);
      %obj.rbm = obj.rbm.addFloatingBase(1, 2, [0;0;0], [0;0;0],'quat'); 
      %obj.rbm = obj.rbm.addFloatingBase(1, 2, [0;0;0], [0;0;0]); 
      obj = obj.compile();
    end

    function obj = compile(obj)
      obj.rbm = obj.rbm.compile();
    end

    function valid = checkConstraints(obj, q)
      valid = checkConstraints@CompositeVertexArrayTree(obj, q);
      valid = valid && obj.isCollisionFree(q);
    end

    function valid = isCollisionFree(obj, q)
      xyz = q(1:3);
      quat = q(4:7); 
      rpy = quat2rpy(quat);
      phi = obj.rbm.collisionDetect([xyz; rpy]);
      valid = all(phi > obj.min_distance);
    end

    function obj = addGeometryToRobot(obj, geom)
      obj.rbm = obj.rbm.addGeometryToBody(2, geom);
    end

    function obj = addGeometryToWorld(obj, geom)
      obj.rbm = obj.rbm.addGeometryToBody(1, geom);
    end

    function obj = setOrientationWeight(obj, orientation_weight)
      obj.weights(2) = orientation_weight;
    end

    function obj = setTranslationSamplingBounds(obj, lb, ub)
      obj.trees{1}.sampling_lb = lb;
      obj.trees{1}.sampling_ub = ub;
    end

    function obj = setLCMGL(obj, varargin)
      obj = setLCMGL@CompositeVertexArrayTree(obj, varargin{:});
      obj.trees{1} = obj.trees{1}.setLCMGL(varargin{:});
    end

    function obj = drawTree(obj, varargin)
      obj.trees{1}.drawTree(varargin{:});
    end

    function drawPath(objA, path_ids_A, objB, path_ids_B)
      if nargin > 2
        drawPath(objA.trees{1}, path_ids_A, objB.trees{1}, path_ids_B);
      else
        drawPath(objA.trees{1}, path_ids_A);
      end
    end
  end
end
