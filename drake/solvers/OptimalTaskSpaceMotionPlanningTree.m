classdef OptimalTaskSpaceMotionPlanningTree < TaskSpaceMotionPlanningTree
  %Subclass of motion planning tree that override and add functions to be
  %used with asymptotically optimal versions of RRT algorithms. It also
  %adds the following properties
%     C             cost array
%     gamma         RRT* parameter
%     steerFactor   radius for the steering function
%     traj          indices of the optimal trajectory
%     eta           radius for tree rewiring
%     costType      type of cost
%     xGoal         

  properties
    C
    gamma
    steerFactor
    traj
    eta
    costType = 'length';
    xGoal
  end
  
  methods
    
    function obj = OptimalTaskSpaceMotionPlanningTree(robot, endEffectorId, endEffectorPoint)
      obj = obj@TaskSpaceMotionPlanningTree(robot, endEffectorId, endEffectorPoint);
    end
    
    function [obj, id_last] = init(obj, q_init)
      obj.C = NaN(1, obj.N);
      obj.C(1) = 0; %An initial cost might be needed if not using path length
      obj.eta = obj.steerFactor;
      
      %Compute Gamma
      R3Tree = obj.trees{obj.tspace_idx}.trees{1};
      freeSpaceVolume = prod(R3Tree.sampling_ub - R3Tree.sampling_lb);
      unitBallVolume = 16/105*pi^3;
      d = 7; %for 7D space
      obj.gamma = (2*(1+1/d))^(1/d)*(freeSpaceVolume/unitBallVolume)^(1/d);
      
      [obj, id_last] = init@TaskSpaceMotionPlanningTree(obj, q_init);
    end
    
    function  [obj, id] = addVertex(obj, q, id_parent, cost)
      [obj, id] = addVertex@TaskSpaceMotionPlanningTree(obj, q, id_parent);
      if nargin < 4
        obj.C(id) = obj.C(id_parent) + obj.cost(id, id_parent);
      else
        obj.C(id) = cost;
      end
    end
    
    function qPath = rebuildTraj(obj, x_goal)      
      if all(obj.getVertex(obj.traj(1)) == x_goal)
        obj.traj = fliplr(obj.traj);
      end
      qPath = [];
      for i = 2:length(obj.traj)
        qPath =  [qPath obj.getVertex(obj.traj(i-1))]; %#ok<AGROW>
        cSpaceTree = obj.trees{obj.cspace_idx};
        qNew = cSpaceTree.getVertex(obj.traj(i-1));
        points = obj.getVertex([obj.traj(i), obj.traj(i-1)]);
        d = obj.trees{obj.tspace_idx}.distanceMetric(points(1:7, 1), points(1:7, 2));
        n = ceil(d/obj.max_edge_length);
        edgePath = NaN(obj.num_vars, n-1);
        for j = 1:n-1
          pt = obj.interpolate(points(:,2), points(:,1), j/n);
          constraints = obj.generateEndEffectorConstraints(pt(1:7));
          qNew = cSpaceTree.solveIK(pt(8:end), qNew, constraints);
          edgePath(:,j) = [pt(1:7); qNew];
        end
        qPath = [qPath edgePath];
      end
      qPath = [qPath x_goal];
    end
    
    function obj = addPointToTraj(obj, point, p1, p2)      
      if obj.getParentId(obj.traj(p2)) == obj.traj(p1)
        obj = obj.addVertex(point, obj.traj(p1));
        obj = obj.setParentId(obj.traj(p2), obj.n);
      else
        obj = obj.addVertex(point, obj.traj(p2));
        obj = obj.setParentId(obj.traj(p1), obj.n);
      end
      obj = obj.getTrajToGoal();
    end
    
    function obj = rrtStarIteration(obj, status, iter, biasSpacing)
      qRand = obj.goalBias(status, [], iter, biasSpacing);
      [qNew, idNearest, valid] = obj.newPoint(qRand, status, iter);
      if valid
        [obj, idNew] = obj.addVertex(qNew, idNearest);
        QNear = obj.near(qNew);
        for q = 1:size(QNear,1)
          if obj.checkEdge(obj.getVertex(QNear(q,1)), obj.getVertex(idNew))
            idMin = QNear(q,1);
            obj.C(idNew) = QNear(q,3);
            break
          end
        end
        if idMin ~= idNearest
          obj = obj.setParentId(idNew, idMin);
        end
        for q = 1:size(QNear,1)
          cost = obj.C(idNew) + obj.cost(idNew, QNear(q, 1));
          if cost + .1 * obj.C(obj.traj(1)) < obj.C(QNear(q, 1)) && obj.checkEdge(obj.getVertex(idNew), obj.getVertex(QNear(q, 1)))
            obj = obj.setParentId(QNear(q, 1), idNew);
            obj = obj.adjustChildrenCost(QNear(q, 1), cost);
            obj.C(QNear(q, 1)) = cost;
            if status == obj.REACHED && any(obj.traj == q)
              obj = obj.getTrajToGoal(obj.traj(1));
            end
          end
        end
      end
    end
    
    function obj = rrtIteration(obj, status, xGoal, iter, biasSpacing)
      qRand = obj.goalBias(status, xGoal, iter, biasSpacing);
      [qNew, idNearest, valid] = obj.newPoint(qRand, status, iter);
      if valid
          obj = obj.addVertex(qNew, idNearest);
      end
    end
    
    function [valid, qPath] = checkEdge(obj, x1, x2)
      
      cSpaceTree = obj.trees{obj.cspace_idx};
      d = obj.trees{obj.tspace_idx}.distanceMetric(x1(obj.idx{obj.tspace_idx}), x2(obj.idx{obj.tspace_idx}));
      n = ceil(d/obj.max_edge_length);
      valid = true;
      
      qNew = x1(obj.idx{obj.cspace_idx});
      qPath = NaN(cSpaceTree.num_vars, n+1);
      qPath(:,1) = x1(obj.idx{obj.cspace_idx});
      
      for j = 1:n-1
        if ~valid
          break
        else
          pt = obj.interpolate(x2, x1, j/n);
          constraints = obj.generateEndEffectorConstraints(pt(1:7));
          [qNew, valid] = cSpaceTree.solveIK(pt(8:end), qNew, constraints);
          if valid
            phi = cSpaceTree.rbm.collisionDetect(qNew, false, cSpaceTree.active_collision_options);
            valid = all(phi > cSpaceTree.min_distance);
          end
        end
        qPath(:,j+1) = qNew;
      end
      qPath(:,end) = x2(obj.idx{obj.cspace_idx});
    end
    
    function qRand = goalBias(obj, status, xGoal, iter, spacing)
      nGoals = size(xGoal, 2);
      goalIdx = mod(iter, spacing*nGoals) == 1:spacing:nGoals*spacing;
      if status ~= obj.REACHED && any(goalIdx)
        qRand = xGoal(:, goalIdx);
      else
        qRand = obj.randomSample();
      end
    end
    
    function qNew = steer(obj, IdNearest, qNew, d)
      if nargin < 4
        d =  obj.distanceMetric(obj.getVertex(IdNearest), qNew);
      end
      if d > obj.steerFactor;
        qPose = qNew(1:7);
        qNew(1:7) = obj.trees{obj.tspace_idx}.interpolate(qPose, obj.trees{obj.tspace_idx}.V(:, IdNearest), (d-obj.steerFactor)/d);
      end
    end
    
    function [qNew, idNearest, valid] = newPoint(obj, qRand, status, iter)
      valid = 0;
      qNew = [];
      idNearest = [];
      if status == obj.REACHED && mod(iter, 4) ~= 1
        subSet = obj.traj;
      else
        subSet = 1:obj.n;
      end
      D = obj.distanceMetric(qRand, obj.getVertex(subSet));
      while ~valid && any(~isnan(D))
        [d, idx] = min(D);
        idNearest = subSet(idx);
        qNearest = obj.trees{obj.cspace_idx}.getVertex(idNearest);
        qNew = obj.steer(idNearest, qRand, d);
        valid = obj.trees{obj.tspace_idx}.isCollisionFree(qNew(1:7));
        if valid
          constraints = obj.generateEndEffectorConstraints(qNew(1:7));
          [qNew(8:end), ~] = obj.trees{obj.cspace_idx}.solveIK(qNearest, qNearest, constraints);
          valid = isValid(obj.trees{obj.cspace_idx}, qNew(8:end));
          valid = valid && obj.checkEdge(obj.getVertex(idNearest), qNew);          
        end
        if ~valid
          D(idx) = NaN;
        end
      end
    end

    function [d, id_near, d_all] = nearestNeighbors(obj, q, status, iter)
      if status == obj.REACHED && mod(iter, 4) ~= 1
        d_all = obj.distanceMetric(q, obj.getVertex(obj.traj));
        [d, id_near] = min(d_all);
        id_near = obj.traj(id_near);
      else
        d_all = obj.distanceMetric(q, obj.getVertex(1:obj.n));
        [d, id_near] = min(d_all);
      end
    end
    
    function valid = CollisionFree(obj, q1, q2)
      qPose_1 = q1(1:7);
      qPose_2 = q2(1:7);
      d = obj.trees{obj.tspace_idx}.distanceMetric(qPose_1, qPose_2);
      n = ceil(d/obj.max_edge_length);
      valid = 1;
      for i = 1:n+1
        pt = obj.trees{obj.tspace_idx}.interpolate(qPose_1, qPose_2, (i-1)/n);
        valid = valid && obj.trees{obj.tspace_idx}.isCollisionFree(pt);
        if ~valid, break; end
      end
    end
    
    function [QNear, q] = near(obj, q)
      qPose = q(1:7);
      d = 7; %for 3D space
      r = min([obj.gamma*(log(obj.n)/obj.n)^(1/d), obj.eta]);
      if obj.n < 2, r = Inf; end;
      QNear = [1:obj.n-1; NaN(1,obj.n-1); NaN(1,obj.n-1)];
      D = obj.trees{obj.tspace_idx}.distanceMetric(qPose, obj.trees{obj.tspace_idx}.getVertex(1:obj.n-1));
      idx = (D-r) <= obj.position_tol;
      QNear = [QNear(1,idx); D(idx); QNear(2,idx)];
      for i = 1:size(QNear, 2)
        QNear(3,i) = obj.C(QNear(1,i)) + obj.cost(QNear(1,i), obj.n);
      end
      QNear = sortrows(QNear', 3);
    end    
    
    function c = cost(obj, idPoint1, idPoint2)
      switch obj.costType
        case 'length'
          c = sqrt(sum((obj.V(1:3, idPoint1) - obj.V(1:3, idPoint2)).^2));
        case 'length+visibility'
          c = sqrt(sum((obj.V(1:3, idPoint1) - obj.V(1:3, idPoint2)).^2)) +...
            0.5 * ~obj.CollisionFree(obj.getVertex(idPoint2), obj.xGoal);
      end
    end
    
    function [obj, cost] = getTrajToGoal(obj, goalId)
      if nargin < 2
        if ~isempty(obj.traj)
          if obj.traj(1) == 1
            goalId = obj.traj(end);
          else
            goalId = obj.traj(1);
          end
        else
          error('If the trajectory is empty the index of the goal vertex must be given\n')
        end
      end
      obj.traj = [];
      while goalId ~= 1
        obj.traj(end+1) = goalId;
        goalId = obj.getParentId(goalId);
      end
      obj.traj(end+1) = 1;  
      cost = obj.C(goalId);
      obj.drawTrajectory();
    end
    
    function drawTrajectory(obj, text)
      if nargin < 2
        text = 'Optimal Trajectory';
      end
      lcmClient = LCMGLClient(text);
      lcmClient.glColor3f(0, 1, 0);
      P = obj.getVertex(obj.traj);
      P = P(1:7, :);
      for p = 1:size(P, 2)
        lcmClient.sphere(P(1:3, p), 0.01, 20, 20);
        if p > 1
          lcmClient.line3(P(1, p), P(2, p), P(3, p), P(1, p-1), P(2, p-1), P(3, p-1));
        end
      end
      lcmClient.switchBuffers();
    end
    
    function obj = shortcut(obj)
      i = 0;
      npts = length(obj.traj);
      nIter =  nchoosek(npts, 2);
      combs = nchoosek(1:npts,2);
      while i < nIter
        checkedComb = randi(nIter - i);
        aIdx = combs(checkedComb, 1);
        bIdx = combs(checkedComb, 2);
        combs(checkedComb, :) = [];
        a = obj.traj(aIdx);
        b = obj.traj(bIdx);
        i = i+1;
        cost = obj.cost(a,b);
        if abs(aIdx-bIdx) > 1 && abs(obj.C(a) - obj.C(b)) > cost && obj.checkEdge(obj.getVertex(a), obj.getVertex(b))
          parent = obj.traj(max([aIdx, bIdx]));
          child = obj.traj(min([aIdx, bIdx]));
          obj = obj.setParentId(child, parent);
          obj.C(child) = obj.C(parent) + cost;
          obj = obj.getTrajToGoal();
          i = 0;
          npts = length(obj.traj);
          nIter =  nchoosek(npts, 2);
          combs = nchoosek(1:npts,2);
        end
      end
    end
    
    function obj = removeVertices(obj, ids)
      obj.C(ids) = [];
      obj.C(end+1:obj.N) = NaN(1, length(ids));
      obj = removeVertices@CompositeVertexArrayTree(obj, ids);
    end
    
    function obj = adjustChildrenCost(obj, idParent, newCost)
      deltaC = newCost - obj.C(idParent);
      queue = idParent;
      done = idParent;
      while ~isempty(queue)
        idParent = queue(end);
        queue = queue(1:end-1);
        obj.C(idParent) = obj.C(idParent) + deltaC;
        children = obj.children{idParent};
        for ch = 1:length(children)
          if all(done ~= children(ch))
            queue = [queue children(ch)];
            done = [done children(ch)];
          end
        end
      end
    end
  end
end