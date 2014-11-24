classdef AllBodiesClosestDistanceConstraint < SingleTimeKinematicConstraint
  % Constraining the closest distance between all bodies to be within
  % [lb,ub]
  % @param lb      -- a scalar, the lower bound of the distance
  % @param ub      -- a scalar, the upper bound of the distance
  % @param tspan   -- a 1x2 vector, the time span of the constraint being
  %                   active
  properties(SetAccess = protected)
    ub
    lb
    active_collision_options
  end

  methods(Access=protected)
    function obj = setNumConstraint(obj)
      kinsol = doKinematics(obj.robot,zeros(obj.robot.getNumPositions(),1));
      phi = evalValidTime(obj,kinsol);
      obj.num_constraint = numel(phi);
    end

    function [c,dc] = evalValidTime(obj,kinsol)
      if ~kinsol.mex
        error('Drake:AllBodiesClosestDistanceConstraint:evalValidTime:NoMexDynamics', ...
          'This method requires a kinsol generated with mex enabled');
      else
        [c,dc] = evalLocal(obj,kinsol);
      end
    end

    function [c,dc] = evalLocal(obj,q_or_kinsol)
      [c,dc] = closestDistance(obj.robot,q_or_kinsol,obj.active_collision_options);
    end
  end
  methods
    function obj = AllBodiesClosestDistanceConstraint(robot,lb,ub,active_collision_options,tspan)
      if(nargin < 5)
        tspan = [-inf inf];
      end
      if nargin < 3 || isempty(active_collision_options) 
        active_collision_options = struct(); 
      end;
      sizecheck(lb,[1,1]);
      sizecheck(ub,[1,1]);
      obj = obj@SingleTimeKinematicConstraint(robot,tspan);
      obj = setNumConstraint(obj);
      assert(obj.num_constraint > 0, ...
        'Drake:AllBodiesClosestDistanceConstraint:noConstraints', ...
        ['You cannot construct an ' ...
         'AllBodiesClosestDistanceConstraint object for a ' ...
         'RigidBodyManipulator that has no collision pairs.']);
      obj.lb = repmat(lb,obj.num_constraint,1);
      obj.ub = repmat(ub,obj.num_constraint,1);
      obj.type = RigidBodyConstraint.AllBodiesClosestDistanceConstraintType;
      if robot.getMexModelPtr~=0 && exist('constructPtrRigidBodyConstraintmex','file')
        obj.mex_ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.AllBodiesClosestDistanceConstraintType,robot.getMexModelPtr,lb,ub,active_collision_options,tspan);
      end
    end

    function cnstr = generateConstraint(obj,t)
      cnstr = generateConstraint@SingleTimeKinematicConstraint(obj,t);
      if ~isempty(cnstr)
        cnstr{1} = setEvalHandle(cnstr{1},@(q,~) obj.evalLocal(q));
      end
    end

    function obj = updateRobot(obj,robot)
      obj.robot = robot;
      obj = setNumConstraint(obj);
      obj.lb = repmat(obj.lb(end),obj.num_constraint,1);
      obj.ub = repmat(obj.ub(end),obj.num_constraint,1);
      obj.mex_ptr = updatePtrRigidBodyConstraintmex(obj.mex_ptr,'robot',robot.getMexModelPtr);
    end

    function [lb,ub] = bounds(obj,t)
      if(obj.isTimeValid(t))
        lb = obj.lb;
        ub = obj.ub;
      else
        lb = [];
        ub = [];
      end
    end

    function [constraintSatisfyFlag,dist,ptsA,ptsB,idxA,idxB] = checkConstraint(obj,q)
      % @retval constraintSatisfyFlag    - A boolean flag, true if the
      %                                    closest distance if within [lb ub]
      % @retval dist                     - if constraint is satisfied, dist = []
      %                                    otherwise, return the dist violates the constraint
      % @retval ptsA                     - if constraint is satisfied, ptsA = []
      %                                    otherwise, return the pts on body A that
      %                                    violate the constraint
      % @retval ptsB                     - if constraint is satisfied, ptsB = []
      %                                    otherwise, return the pts on body B that
      %                                    violate the constraint
      % @retval idxA                     - if constraint is satisfied, idxA = []
      %                                    otherwise, return bodyA index that violate the
      %                                    constraint
      % @retval idxB                     - if constraint is satisfied, idxB = []
      %                                    otherwise, return bodyB index that violate the
      %                                    constraint
      kinsol = doKinematics(obj.robot,q);
      [dist,normal,d,xA,xB,idxA,idxB] = contactConstraints(obj.robot,kinsol);
      unsatisfy_idx = reshape((dist > obj.ub) | (dist < obj.lb),1,[]);
      if(~any(unsatisfy_idx))
        constraintSatisfyFlag = true;
        dist = [];
        ptsA = [];
        ptsB = [];
        idxA = [];
        idxB = [];
      else
        constraintSatisfyFlag = false;
        dist = dist(unsatisfy_idx);
        ptsA = zeros(3,sum(unsatisfy_idx));
        ptsB = zeros(3,sum(unsatisfy_idx));
        i = 1;
        for idx = find(unsatisfy_idx)
          ptsA(:,i) = forwardKin(obj.robot,kinsol,idxA(idx),xA(:,idx));
          ptsB(:,i) = forwardKin(obj.robot,kinsol,idxB(idx),xB(:,idx));
          i = i+1;
        end
        idxA = idxA(unsatisfy_idx);
        idxB = idxB(unsatisfy_idx);
      end
    end

    function drawConstraint(obj,q,lcmgl)
      checkDependency('lcmgl');
      [~,~,ptsA,ptsB,idxA,idxB] = obj.checkConstraint(q);

      for i = 1:size(ptsA,2);
        lcmgl.glColor3f(0,0,0); % black

        lcmgl.text(ptsA(:,i),obj.robot.getLinkName(idxA(i)),0,0);
        lcmgl.text(ptsB(:,i),obj.robot.getLinkName(idxB(i)),0,0);

        lcmgl.glBegin( lcmgl.LCMGL_LINES);
        lcmgl.glVertex3f(ptsA(1,i), ptsA(2,i), ptsA(3,i));
        lcmgl.glVertex3f(ptsB(1,i), ptsB(2,i), ptsB(3,i));
        lcmgl.glEnd();

        lcmgl.glColor3f(1,0,0); % red

        lcmgl.sphere(ptsA(:,i),.01,20,20);
        lcmgl.sphere(ptsB(:,i),.01,20,20);

        lcmgl.glColor3f(.7,.7,.7); % gray
      end

      lcmgl.switchBuffers();
    end

    function name = name(obj,t)
      if(obj.isTimeValid(t))
        name = repmat({'Closest distance constraint for all non-adjacent bodies'},obj.getNumConstraint(t),1);
      else
        name = [];
      end
    end

  end
end
