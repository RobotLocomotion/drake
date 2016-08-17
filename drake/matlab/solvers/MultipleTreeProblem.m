classdef MultipleTreeProblem
  
  properties
    robot
    endEffectorId
    xStart
    xGoal
    endEffectorPoint
    additionalConstraints
    qNom
    mergingThreshold
    minDistance
    trees
    nTrees
    startPoints
    activeCollisionOptions
    status
    iterations
  end
  
  properties (Constant)
    EXPLORING = 1
    GOAL_REACHED = 2
    SUCCESS = 1
    FAIL_ITERATION_LIMIT = 11
    FAIL_NO_FINAL_POSE = 12
  end
  
  methods
    
    function obj = MultipleTreeProblem(robot, endEffectorId,...
        xStart, xGoal, xStartAddTrees, additionalConstraints, qNom, varargin)
      % Class which builds and maintains multiple motion planning trees
      % (i.e. two or more) and searches to connect them such that a
      % spanning solution can reach a goal
      % Currently uses the RRT* algorithm in the main iteration step.
      
      warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
      opt = struct('mergingthreshold', 0.2,...
        'mindistance', 0.005,'activecollisionoptions', struct(), 'ikoptions', struct(),...
        'steerfactor', 0.1, 'orientationweight', 1, 'maxedgelength', 0.05,...
        'angletol', 10*pi/180, 'positiontol', 1e-3, 'endeffectorpoint', [0; 0; 0]);
      optNames = fieldnames(opt);
      nArgs = length(varargin);
      if round(nArgs/2)~=nArgs/2
        error('Needs propertyName/propertyValue pairs')
      end
      for pair = reshape(varargin,2,[])
        inpName = lower(pair{1});
        if any(strcmp(inpName,optNames))
          opt.(inpName) = pair{2};
        else
          error('%s is not a recognized parameter name',inpName)
        end
      end
      
      obj.robot = robot;
      obj.endEffectorId = endEffectorId;
      obj.xStart = xStart;
      obj.xGoal = xGoal;
      obj.nTrees = size(xStartAddTrees, 2) + 2;
      obj.additionalConstraints = additionalConstraints;
      obj.qNom = qNom;
      obj.mergingThreshold = opt.mergingthreshold;
      obj.minDistance = opt.mindistance;
      obj.activeCollisionOptions = opt.activecollisionoptions;
      obj.endEffectorPoint = opt.endeffectorpoint;
      obj.status = obj.EXPLORING;
      
      %compute sampling space
      xyzMin = [min([xStart(1:2), xGoal(1:2)], [], 2) - 0.5; 0];
      xyzMax = [max([xStart(1:2), xGoal(1:2)], [], 2) + 0.5; max([xStart(3), xGoal(3)]) + 1];
      
      %initialize trees
      obj.trees = OptimalTaskSpaceMotionPlanningTree.empty(obj.nTrees, 0);  
      for t = 1:obj.nTrees
        obj.trees(t) = OptimalTaskSpaceMotionPlanningTree(obj.robot, obj.endEffectorId, obj.endEffectorPoint);
        obj.trees(t).steerFactor = opt.steerfactor;
        obj.trees(t) = obj.trees(t).setMinDistance(obj.minDistance);
        obj.trees(t) = obj.trees(t).setOrientationWeight(opt.orientationweight);
        obj.trees(t).max_edge_length = opt.maxedgelength;
        obj.trees(t).max_length_between_constraint_checks = opt.maxedgelength;
        obj.trees(t).angle_tol = opt.angletol;
        obj.trees(t).position_tol = opt.positiontol;
        obj.trees(t).trees{obj.trees(t).cspace_idx}.active_collision_options = opt.activecollisionoptions;
        obj.trees(t).trees{obj.trees(t).cspace_idx}.ikoptions = opt.ikoptions;
        obj.trees(t) = obj.trees(t).setTranslationSamplingBounds(xyzMin, xyzMax);
        obj.trees(t) = obj.trees(t).addKinematicConstraint(obj.additionalConstraints{:});
        obj.trees(t) = obj.trees(t).setNominalConfiguration(obj.qNom);
        obj.trees(t) = obj.trees(t).compile();
        if t == 1
          obj.trees(t) = obj.trees(t).setLCMGL('Tree 1 (Start Pose)',[1,0,0]);
        elseif t == 2
          obj.trees(t) = obj.trees(t).setLCMGL('Tree 2 (Goal Pose)',[0,0,1]);
        else
          obj.trees(t) = obj.trees(t).setLCMGL(sprintf('Tree %d', t),[1,0,1]);
        end
      end
      assert(obj.trees(1).isValid(xStart))
      
      %set tree starting points
      obj.startPoints = [obj.xStart, zeros(obj.robot.num_positions + 7, 1), xStartAddTrees];
      %obj.startPoints = [obj.xStart, obj.xGoal, xStartAddTrees];
    end
    
    function [obj, info, cost, qPath] = rrtStar(obj, options, xGoal)
      if nargin < 2, options = struct(); end
      
      info = obj.SUCCESS;
      qPath = [];
      cost = [];
      obj.xGoal = xGoal;
      obj.startPoints(:,2) = obj.xGoal;
      
      %Set and apply defaults
      defaultOptions.N = 1000;
      defaultOptions.visualize = true;
      defaultOptions.firstFeasibleTraj = false;
      options = applyDefaults(options, defaultOptions);      
      
      for treeIdx = obj.nTrees:-1:3
        if ~obj.trees(treeIdx).isValid(obj.startPoints(:, treeIdx))
          obj = obj.deleteTree(treeIdx);
        end
      end
      for treeIdx = 1:obj.nTrees
          obj.trees(treeIdx).costType = options.costType;
          obj.trees(treeIdx) = obj.trees(treeIdx).init(obj.startPoints(:, treeIdx));
      end
      
      %Iteration loop
      obj.iterations = zeros(1, obj.nTrees);
      for iter = 1:options.N
        %Check if the goal has been reached
        xLast = obj.trees(treeIdx).getVertex(obj.trees(treeIdx).n);
        if obj.status ~= obj.GOAL_REACHED
          [d, idxNearest, treeIdxNearest] = nearestNeighbor(obj, xLast, treeIdx);
          xNearest = obj.trees(treeIdxNearest).getVertex(idxNearest);
          if d < obj.mergingThreshold && obj.trees(treeIdxNearest).CollisionFree(xLast, xNearest)
            if any(treeIdx == [1 2])
              [obj, prevRoot] = obj.mergeTrees(idxNearest, obj.trees(treeIdx).n, treeIdxNearest, treeIdx, options);
            else
              [obj, prevRoot] = obj.mergeTrees(obj.trees(treeIdx).n, idxNearest, treeIdx, treeIdxNearest, options);
            end
            %If the merged trees are the start and goal tree
            if all(ismember([treeIdx, treeIdxNearest], [1, 2]))
              obj.status = obj.GOAL_REACHED;
              obj.trees(1) = obj.trees(1).getTrajToGoal(prevRoot);
              obj.trees(1).eta = 0.5;
              lastCost = obj.trees(1).C(obj.trees(1).traj(1));
              lastUpdate = obj.trees(1).n;
            end
          end
        end
        
        %if goal has been reached
        if obj.status == obj.GOAL_REACHED
          %if the cost improves recompute trajectory
          if obj.trees(1).C(obj.trees(1).traj(1)) < lastCost
            obj.trees(1) = obj.trees(1).getTrajToGoal(obj.trees(1).traj(1));
            lastCost = obj.trees(1).C(obj.trees(1).traj(1));
            lastUpdate = obj.trees(1).n;
          end
          %break the loop if the cost doesn't improve after having added 10 points
          if options.firstFeasibleTraj || lastUpdate > 0 && obj.trees(1).n - lastUpdate > 10
            break
          end
          treeIdx = 1;
        else
          treeIndices = find([obj.trees.n] == min([obj.trees.n]));
          treeIdx = treeIndices(randi(length(treeIndices)));
          xGoals = NaN(obj.trees(1).num_vars, obj.nTrees);
          for i = find(1:obj.nTrees ~= treeIdx)
            xGoals(:,i) = obj.trees(i).getCentroid();
          end
          xGoals(:,treeIdx) = [];
        end
        
        obj.iterations(treeIdx) = obj.iterations(treeIdx) + 1;
        if obj.status == obj.GOAL_REACHED
          obj.trees(treeIdx) = rrtStarIteration(obj.trees(treeIdx), obj.status, obj.iterations(treeIdx), floor(20/obj.nTrees));
        else
          obj.trees(treeIdx) = rrtIteration(obj.trees(treeIdx), obj.status, xGoals, obj.iterations(treeIdx), floor(20/obj.nTrees));
        end
        %update the tree plot
        if options.visualize
          obj.trees(treeIdx) = obj.trees(treeIdx).drawTree();
          drawnow
        end
      end
      if obj.status == obj.GOAL_REACHED
        obj.trees(1) = obj.trees(treeIdx);
        obj.trees(1).setLCMGL(obj.trees(1).lcmgl_name, obj.trees(1).line_color);
        obj.trees(1) = obj.trees(1).shortcut();
        qPath = obj.trees(1).rebuildTraj(obj.xGoal);
        cost = obj.trees(1).C(obj.trees(1).traj(1));
        if options.visualize
          fprintf('Final Cost = %.4f\n', cost)
        end
      else
        info = obj.FAIL_ITERATION_LIMIT;
      end
    end
    
    function [obj, prevRoot] = mergeTrees(obj, ptAidx, ptBidx, treeAidx, treeBidx, options)
      %Merge tree A into B and delete tree A
      lastAdded = NaN;
      nextToAdd = ptAidx;
      nextParent = ptBidx;
      while lastAdded ~= 1
        obj.trees(treeBidx) = obj.trees(treeBidx).addVertex(obj.trees(treeAidx).getVertex(nextToAdd), nextParent);
        if options.visualize
          obj.trees(treeBidx) = obj.trees(treeBidx).drawTree();
        end
        nextParent = obj.trees(treeBidx).n;
        queue = [nextToAdd; nextParent];
        done = nextToAdd;
        while ~isempty(queue)
          idParentA = queue(1, end);
          idParentB = queue(2, end);
          queue = queue(:, 1:end-1);
          children = obj.trees(treeAidx).getChildren(idParentA);
          children = children(children ~= lastAdded);
          for ch = 1:length(children)
            if all(done ~= children(ch))
              obj.trees(treeBidx) = obj.trees(treeBidx).addVertex(obj.trees(treeAidx).getVertex(children(ch)), idParentB);
              if options.visualize
                obj.trees(treeBidx) = obj.trees(treeBidx).drawTree();
              end
              queue = [queue [children(ch); obj.trees(treeBidx).n]];
              done = [done children(ch)];
            end
          end
        end
        lastAdded = nextToAdd;
        nextToAdd = obj.trees(treeAidx).getParentId(lastAdded);
      end
      obj = obj.deleteTree(treeAidx);
      prevRoot = nextParent;
    end
    
    function obj = deleteTree(obj, treeIdx)
      tree = obj.trees(treeIdx);
      tree.setLCMGL(tree.lcmgl_name, tree.line_color);
      obj.trees(treeIdx) = [];
      obj.startPoints(:, treeIdx) = [];
      obj.nTrees = obj.nTrees - 1;
    end
    
    function [d, idNearest, treeNearest] = nearestNeighbor(obj, q, treeIdx)
      d = Inf;
      idNearest = NaN;
      treeNearest = NaN;
      for t = 1:obj.nTrees
        if t ~= treeIdx
          [newd, newId] = obj.trees(t).nearestNeighbor(q);
          if newd < d
            d = newd;
            idNearest = newId;
            treeNearest = t;
          end
        end
      end
    end
        
    function J = computeJacobian(obj, kinSol, joints, endEffector)
      np = obj.robot.num_positions;
      nj = length(joints);
      J = zeros(6, nj);
      T = kinSol.T{endEffector}(1:3, 1:3);
      J(1,:) = kinSol.dTdq{endEffector}(joints,4)';
      J(2,:) = kinSol.dTdq{endEffector}(np + joints,4)';
      J(3,:) = kinSol.dTdq{endEffector}(np*2 + joints,4)';
      for j = 1:nj
        S = T' * kinSol.dTdq{endEffector}([joints(j),np+joints(j),2*np+joints(j)],1:3);        
        J(4:6, j) = T * [S(3,2);S(1,3);S(2,1)];
      end
    end

    function valid = checkConstraints(obj, q, constraints)
      valid = true;
      kinsol = obj.robot.doKinematics(q);
      tol = 1e-3;
      for i = 1:numel(constraints)
        if isa(constraints{i}, 'QuasiStaticConstraint')
          valid = valid && constraints{i}.checkConstraint(kinsol);
        else
          if valid
            [lb, ub] = constraints{i}.bounds(0);
            if isa(constraints{i}, 'PostureConstraint')
              y = q;
            else
              y = eval(constraints{i}, 0, kinsol);
            end
            valid = all(y - lb > -tol) && all(ub - y > -tol);
          end
        end
      end
    end
    
  end
end