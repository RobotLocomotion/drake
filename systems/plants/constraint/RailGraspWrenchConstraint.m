classdef RailGraspWrenchConstraint < ContactWrenchConstraint
  % Constrain the force and torque at the contact point when the hand is grasping a rail. The force is subject
  % to magnitude constraint and friction cone constraint. The bounds on the torque is a
  % cylinder co-axial with the rail
  % @param body               -- The index of contact body
  % @param body_name          -- The name of the body
  % @param grasp_center_pt     -- A 3 x 1 double matrix, a point on the body frame that is
  % the center of the grasping
  % @param grasp_length  -- A non negative scalar. The length of the grasping region
  % projected onto the rail
  % @param rail_axis   -- A 3 x 1 vector, the axis of the rail
  % @param rail_radius -- A non-negative scalar, the radius of the rail
  % @param rail_fc_mu  -- A non-negative scalar, the friction coefficient on the rail
  % @param force_max    -- A positive scalar, the maximum maginitude the force
  properties(SetAccess = protected)
    body
    body_name
    grasp_center_pt
    grasp_length
    num_pts
    rail_axis
    rail_radius
    rail_fc_mu
    force_ub;
    force_max;
  end
  
  properties(SetAccess = protected, GetAccess = protected)
    % @param torque_bnd_height   -- A non-negative double scalar. The height of the
    % cylinder that bounds the torque
    % @param torque_bnd_radius   -- A non-negative double scalar. The radius of the
    % cylinder that bounds the torque
    torque_bnd_height
    torque_bnd_radius
  end
  
  methods
    function obj = RailGraspWrenchConstraint(robot,body,grasp_center_pt,grasp_length,rail_axis,rail_radius,rail_fc_mu,force_max,tspan)
      if(nargin<9)
        tspan = [-inf,inf];
      end
      obj = obj@ContactWrenchConstraint(robot,tspan);
      body_size = size(body);
      if(~isnumeric(body) || length(body_size) ~= 2 || body_size(1) ~= 1 || body_size(2) ~= 1)
        error('Drake:RailGraspWrenchConstraint: body should be a numeric scalar');
      end
      obj.body = body;
      obj.body_name = obj.robot.getBody(obj.body).linkname;
      pt_size = size(grasp_center_pt);
      if(~isnumeric(grasp_center_pt) || pt_size(1) ~= 3 || pt_size(2) ~= 1)
        error('Drake:RailGraspWrenchConstraint: grasp_center_pt should be a 3 x 1 vector');
      end
      obj.grasp_center_pt = grasp_center_pt;
      obj.num_pts = 1;
      if(~isnumeric(grasp_length) || numel(grasp_length) ~= 1 || grasp_length<0)
        error('Drake:RailGraspWrenchConstraint: grasp_length should be a non-negative scalar');
      end
      rail_axis_size = size(rail_axis);
      if(~isnumeric(rail_axis_size) || rail_axis_size(1) ~= 3 || rail_axis_size(2) ~= 1)
        error('Drake:RailGraspWrenchConstraint: rail_axis should be a 3 x 1 vector');
      end
      rail_axis_norm = norm(rail_axis);
      if(rail_axis_norm<1e-10)
        error('Drake:RailGraspWrenchConstraint: rail_axis should be a non-zero vector');
      end
      obj.rail_axis = rail_axis/rail_axis_norm;
      if(~isnumeric(rail_radius) || numel(rail_radius) ~= 1 || rail_radius<0)
        error('Drake:RailGraspWrenchConstraint: rail_radius should be a non-negative scalar');
      end
      obj.rail_radius = rail_radius;
      if(~isnumeric(rail_fc_mu) || numel(rail_fc_mu) ~= 1 || rail_fc_mu<0)
        error('Drake:RailGraspWrenchConstraint: rail_fc_mu should be a non-negative scalar');
      end
      obj.rail_fc_mu = rail_fc_mu;
      if(~isnumeric(force_max) || numel(force_max) ~= 1 || force_max <= 0)
        error('Drake:RailGraspWrenchConstraint: force_max should be a positive scalar');
      end
      obj.torque_bnd_height = obj.force_max*obj.rail_radius;
      obj.torque_bnd_radius = obj.force_max*obj.grasp_length/2;
      obj.num_constraint = 4;
      obj.F_size = [6,1];
      obj.type = RigidBodyConstraint.RailGraspWrenchConstraint;
      obj.F_lb = -[obj.force_max*ones(3,1);sqrt(obj.torque_bnd_height^2+obj.torque_bnd_radius^2)*ones(3,1)];
      obj.F_ub = [obj.force_max*ones(3,1);sqrt(obj.torque_bnd_height^2+obj.torque_bnd_radius^2)*ones(3,1)];
    end
    
    function [c,dc_val] = evalSparse(obj,t,kinsol,F)
      % This function evaluates the friction cone constraint on the force,magnitude
      % constraint on the force, and cylinder constraint on the torque.
      % @param t       - A scalar, the time to evaluate friction cone constraint
      % @param kinsol  - kinematics tree returned from doKinematics function
      % @param F       - A 6 x 1 double vector. The contact forces and torque about the
      % contact point
      % @retval c      - A 4 x 1 double vector, the constraint value
      % @retval dc_val - A double column vector. The nonzero entries of constraint gradient w.r.t F 
      if(obj.isTimeValid(t))
      f_mag = norm(F(1:3));
      df_magdforce = F(1:3)'/f_mag;
      fc_cnst = F(1:3)'*obj.rail_axis/f_mag;
      dfc_cnstdforce = (obj.rail_axis'*f_mag-F(1:3)'*obj.rail_axis*df_magdforce)/f_mag^2;
      tau = F(4:6);
      tau_axis = tau'*obj.rail_axis;
      dtau_axisdtau = obj.rail_axis';
      tau_radius = norm(tau-tau_axis*obj.rail_axis);
      else
        c = [];
        dc = []'
      end
    end
  end
end