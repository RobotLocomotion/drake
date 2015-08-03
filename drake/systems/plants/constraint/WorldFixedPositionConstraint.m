classdef WorldFixedPositionConstraint < MultipleTimeKinematicConstraint
  % Constraint some points on a certain body to be in a fixed position for
  % a time interval
  % @param robot             -- A RigidBodyManipulator or a
  %                             TimeSteppingRigidBodyManipulator
  % @param body              -- An int scalar, the body index
  % @param pts               -- A 3 x npts double array, pts(:,i) is the i'th
  %                             point
  % @param tspan             -- Optional input, a 1x2 double array, the time span
  %                             of this constaint being active. Default is
  %                             [-inf inf]
  
  properties(SetAccess = protected)
    body
    body_name;
    pts
    tol = 0; % tolerance for the constraint
  end
  
  methods(Access = protected)
    function [c,dc_valid] = evalValidTime(obj,kinsol_cell)
      N = length(kinsol_cell);
      nq = obj.robot.getNumPositions();
      n_pts = size(obj.pts,2);
      pos = zeros(3*n_pts,N);
      if(nargout == 2)
        J = zeros(3*n_pts*N,nq);
      end
      for i = 1:N
        kinsol = kinsol_cell{i};
        if(nargout == 1)
          pos_tmp = forwardKin(obj.robot,kinsol,obj.body,obj.pts,0);
        elseif(nargout == 2)
          [pos_tmp,J(3*n_pts*(i-1)+(1:3*n_pts),:)] = forwardKin(obj.robot,kinsol,obj.body,obj.pts,0);
        end
        pos(:,i) = pos_tmp(:);
      end
      diff_pos = diff(pos,[],2);
      diff_pos = [diff_pos pos(:,1)-pos(:,N)];
      c1 = sum(diff_pos.*diff_pos,2);
      c = (sum(reshape(c1,3,n_pts),1))';
      if(nargout == 2)
        dc1 = reshape(permute(reshape((bsxfun(@times,reshape((4*pos-2*[pos(:,2:end) pos(:,1)]-2*[pos(:,end) pos(:,1:end-1)]),[],1),ones(1,nq))...
          .*J)',nq,3*n_pts,N),[2,1,3]),3*n_pts,nq*N);
        dc_valid = reshape((sum(reshape(permute(reshape(dc1',nq*N,3,n_pts),[2,1,3]),3,nq*N*n_pts),1))',nq*N,n_pts)';

      end
    end
  end
  
  methods
    function obj = WorldFixedPositionConstraint(robot,body,pts,tspan)
      if nargin == 3
        tspan = [-inf inf];
      end
      mex_ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.WorldFixedPositionConstraintType,robot.getMexModelPtr,body,pts,tspan);
      obj = obj@MultipleTimeKinematicConstraint(robot,tspan);
      sizecheck(body,[1,1]);
      if(~isnumeric(body))
        error('Drake:WorldFixedPositionConstraint: body must be an integer');
      end
      obj.body = floor(body);
      obj.body_name = obj.robot.getBodyOrFrameName(obj.body);
      sizecheck(pts,[3,nan]);
      if(~isnumeric(pts))
        error('Drake:WorldFixedPositionConstraint: pts must be a double array');
      end
      if(any(isnan(pts)|(isinf(pts))))
        error('Drake:WorldFixedPositionConstraint: pts cannot have nan or inf');
      end
      if(size(pts,2)>2)
        warning('Drake:WordFixedPositionConstraint: you have more than 3 points in fixed positions, consider to use WorldFixedBodyPostureConstraint, which is more efficient');
      end
      obj.pts = pts;
      obj.type = RigidBodyConstraint.WorldFixedPositionConstraintType;
      obj.mex_ptr = mex_ptr;
    end
    
    function num = getNumConstraint(obj,t)
      valid_t = t(obj.isTimeValid(t));
      if(length(valid_t)>=2)
        num = size(obj.pts,2);
      else
        num = 0;
      end
    end
    
    function [lb,ub] = bounds(obj,t,N)
      % [lb,ub] = bounds(obj,t) returns the upper and lower bounds for this
      % constraint at all valid times given in t.
      %
      % [lb,ub] = bounds(obj,[],N) returns the upper and lower bounds for this
      % constraint for N valid times
      %
      % @param obj  -- WorldFixedPositionConstraint object
      % @param t    -- Vector of times
      % @param N    -- Integer number of time points
      if isempty(t)
        if(N>=2)
          n_pts = size(obj.pts,2);
          lb = -obj.tol*ones(n_pts,1);
          ub = obj.tol*ones(n_pts,1);
        else
          lb = [];
          ub = [];
        end
      else
        valid_t = t(obj.isTimeValid(t));
        if(length(valid_t)>=2)
          n_pts = size(obj.pts,2);
          lb = -obj.tol*ones(n_pts,1);
          ub = obj.tol*ones(n_pts,1);
        else
          lb = [];
          ub = [];
        end
      end
    end
    
    % set the tolerance for the constraint, default is 0.
    function obj = setTol(obj,tol)
      if ~isnumeric(tol)
        error('Drake:WorldFixedPositionConstraint:TypeError','tol must be a numeric type');
      end

      if ~isscalar(tol)
        error('Drake:WorldFixedPositionConstraint:SizeError','tol must be a scalar');
      end

      if tol < 0
        error('Drake:WorldFixedPositionConstraint:SignError','tol must be >= 0');
      end
      
       obj.tol = tol; 
    end
    
    function name_str = name(obj,t)
      valid_t = t(obj.isTimeValid(t));
      if(length(valid_t)>=2)
        n_pts = size(obj.pts,2);
        name_str = cell(n_pts,1);
        for i = 1:n_pts
          name_str{i} = sprintf('World fixed position constraint for %s %ds points',obj.body_name,i);
        end
      else
        name_str = {};
      end
    end

    function joint_idx = kinematicPathJoints(obj)
      [~,joint_path] = obj.robot.findKinematicPath(1,obj.body);
      joint_idx = vertcat(obj.robot.body(joint_path).position_num)';
    end
  end
end
