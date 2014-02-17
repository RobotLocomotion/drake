classdef FrictionConeWrenchConstraint < ContactWrenchConstraint
  % constrain the friction force to be within a friction cone.
  % @param body       -- The index of contact body on the robot
  % @param body_pts   -- A 3 x num_pts double matrix, each column represents the coordinate
  % of the contact point on the body frame.
  % @param num_pts    -- The number of contact points
  % @param FC_mu      -- A 1 x num_pts double vector, FC_mu(i) is the friction coefficient
  % of the friction cone at contact point body_pts(:,i)
  % @param FC_axis    -- A 3 x num_pts double matrix, FC_axis(:,i) is the axis of the
  % friction cone at the contact point body_pts(:,i). FC_axis(:,i) is in the world frame
  properties(SetAccess = protected)
    body
    body_pts
    num_pts
    FC_mu
    FC_axis
  end
  
  properties(SetAccess = protected, GetAccess = protected)
    FC_lb;
    FC_ub;
  end
  
  methods
    function obj = FrictionConeWrenchConstraint(robot,body,body_pts,FC_mu,FC_axis,tspan)
      % @param body       -- The index of contact body on the robot
      % @param body_pts   -- A 3 x num_pts double matrix, each column represents the coordinate
      % of the contact point on the body frame.
      % @param FC_mu      -- A 1 x num_pts double vector, FC_mu(i) is the friction coefficient
      % of the friction cone at contact point body_pts(:,i)
      % @param FC_axis    -- A 3 x num_pts double matrix, FC_axis(:,i) is the axis of the
      % friction cone at the contact point body_pts(:,i). FC_axis(:,i) is in the world frame
      if(nargin<6)
        tspan = [-inf inf];
      end
      obj = obj@ContactWrenchConstraint(robot,tspan);
      body_size = size(body);
      if(~isnumeric(body) || length(body_size) ~= 2 || body_size(1) ~= 1 || body_size(2) ~= 1)
        error('Drake:FrictionConeWrenchConstraint: body should be a numeric scalar');
      end
      obj.body = body;
      body_pts_size = size(body_pts);
      if(~isnumeric(body_pts) || length(body_pts_size) ~= 2 || body_pts_size(1) ~= 3)
        error('Drake:FrictionConeWrenchConstraint: body_pts should be 3 x num_pts double matrix');
      end
      obj.body_pts = body_pts;
      obj.num_pts = body_pts_size(2);
      mu_size = size(FC_mu);
      if(length(mu_size) == 2 && mu_size(1) == 1 && mu_size(2) == 1)
        FC_mu = FC_mu*ones(1,obj.num_pts);
      end
      if(~isnumeric(FC_mu) || any(FC_mu<0) || length(mu_size) ~= 2 || mu_size(1) ~= 1 || mu_size(2) ~= obj.num_pts)
        error('Drake:FrictionConeWrenchConstraint: FC_mu should be a non-negative 1 x num_pts array');
      end
      obj.FC_mu = FC_mu;
      axis_size = size(FC_axis);
      if(length(axis_size) == 2 && axis_size(1) == 3 && axis_size(2) == 1)
        FC_axis = bsxfun(@times,FC_axis,ones(1,obj.num_pts));
      end
      if(~isnumeric(FC_axis) || length(axis_size) ~= 2 || axis_size(1) ~= 1 || axis_size(2) ~= obj.num_pts)
        error('Drake:FrictionConeWrenchConstraint: FC_axis should be a 3 x num_pts array');
      end
      FC_axis_norm = sqrt(sum(FC_axis.^2,1));
      obj.FC_axis = FC_axis./(bsxfun(@times,FC_axis_norm,ones(3,1)));
      obj.type = RigidBodyConstraint.FrictionConeWrenchConstraint;
      obj.FC_lb = ones(1,obj.num_pts)./sqrt(obj.FC_mu.^2+ones(1,obj.num_pts));
      obj.FC_ub = ones(1,obj.num_pts);
    end
    
    
    function [c,dc] = eval(obj,t,F)
      if(obj.isTimeValid(t))
        valid_F = checkForceSize(obj,F);
        if(~valid_F)
          error('Drake:FrictionConeWrenchConstraint:friction force should be 3 x n_pts matrix');
        end
        F_norm = sqrt(sum(F.^2,1));
        c = reshape(sum(F.*obj.FC_axis,1)./F_norm,[],1);
        dc_entry = (obj.FC_axis'.*bsxfun(@times,F_norm.^2,ones(1,3))-bsxfun(@times,sum(F'.*obj.FC_aixs',2),ones(1,3)).*F')./bsxfun(@times,F_norm.^3,ones(1,3));
        dc = sparse(reshape(bsxfun(@times,ones(3,1),(1:obj.num_pts)),[],1),(1:3*obj.num_pts)',reshape(dc_entry',[],1),obj.num_pts,obj.num_pts*3);
      else
        c = [];
        dc = [];
      end
    end
    
    function [lb,ub] = bounds(obj,t)
      if(obj.isTimeValid(t))
        lb = obj.FC_lb;
        ub = obj.FC_ub;
      else
        lb = [];
        ub = [];
      end
    end
    
    function [w,dw] = wrench(obj,t,kinsol,F)
      % Compute the wrench from contact position and contact force
      % @param F     -- a 3 x num_pts double matrix. F(:,i) is the contact force at
      % i'th contact point
      % @retval w    -- a 6 x num_pts double matrix. w(:,i) is the wrench ([force;torque])
      % i'th contact point
      % @retval dw   -- a 6*num_pts x nq+3*num_pts double matrix. The gradient of wrench w
      % w.r.t joint angles q and contact force F
      if(obj.isTimeValid(t))
        valid_F = checkForceSize(obj,F);
        if(~valid_F)
          error('Drake:FrictionConeWrenchConstraint:friction force should be 3 x n_pts matrix');
        end
        nq = obj.robot.getNumDOF();
        [body_pos,dbody_pos] = forwardKin(obj.robot,kinsol,obj.body,obj.body_pts,0);
        tau = cross(body_pos,F,1);
        w = [F;tau];
        dtau = zeros(obj.num_pts*3,6);
        dtaudq = zeros(obj.num_pts*3,nq);
        for i = 1:obj.num_pts
          dtau((i-1)*3+(1:3),:) = dcross(body_pos(:,i),F(:,i));
          dtaudq((i-1)*3+(1:3),:) = dtau((i-1)*3+(1:3),1:3)*dbody_pos((i-1)*3+(1:3),:);
        end
        dtaudF = dtau(:,4:6);
        dw_entery = 
      else
        w = [];
        dw = [];
      end
    end
  end
  
  methods(Access = protected)
    function flag = checkForceSize(obj,F)
      F_size = size(F);
      flag = isnumeric(F) && length(F_size) == 2 && F_size(1) == 3 && F_size(2) == obj.num_pts;
    end
  end
end